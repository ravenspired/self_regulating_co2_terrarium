from machine import Pin, UART, I2C, SoftI2C
from gpio_lcd import GpioLcd
from bmp085 import BMP180

import time




# Global vars
MHZ19B_warmupsecs = 10 # Should be 180 seconds (3 minutes) for MH-Z19B
CO2_threshold = 1000 # CO2 programmed determined threshold for turning on the pump

# Set up the barometers
i2c1 = SoftI2C(sda = Pin(9), scl = Pin(8), freq = 100000) 
bmp1 = BMP180(i2c1)        
bmp1.oversample = 2
bmp1.sealevel = 101325

i2c2 = SoftI2C(sda = Pin(3), scl = Pin(2), freq = 100000) 
bmp2 = BMP180(i2c2)        
bmp2.oversample = 2
bmp2.sealevel = 101325

i2c3 = SoftI2C(sda = Pin(6), scl = Pin(7), freq = 100000) 
bmp3 = BMP180(i2c3)        
bmp3.oversample = 2
bmp3.sealevel = 101325


def get_tempC(sensor, times):
    temps_C = []
    divisor = times
    for i in range(times):
        try:
            temps_C.append(sensor.temperature)
        except:
            divisor -= 1
    if divisor == 0:
        return 0
    return sum(temps_C)/divisor

def get_pres_hPa(sensor, times):
    pres_hPa = []
    divisor = times
    for i in range(times):
        try:
            pres_hPa.append(sensor.pressure)
        except:
            divisor -= 1
    if divisor == 0:
        return 0
    return sum(pres_hPa)/divisor

def get_altitude(sensor, times):
    altitude = []
    divisor = times
    for i in range(times):
        try:
            altitude.append(sensor.altitude)
        except:
            divisor -= 1
    if divisor == 0:
        return 0
    return sum(altitude)/divisor



# Set up the LCD
lcd = GpioLcd(rs_pin=Pin(16),
              enable_pin=Pin(17),
              d4_pin=Pin(18),
              d5_pin=Pin(19),
              d6_pin=Pin(20),
              d7_pin=Pin(21),
              num_lines=2, num_columns=16)

# Set up the LCD Backlight
led = Pin(22, Pin.OUT)

def backlight_on():
    led.value(1)
    

def backlight_off():
    led.value(0)

# Set up the pushbutton HID
button = Pin(28, Pin.IN, Pin.PULL_UP)   #Internal pull-up
                    
                    
def check_button_press():
    if button.value() == 0:       #key press
        return True     
    else:
        return False

# Set up the motor control
IN1 = Pin(27, Pin.OUT)
IN2 = Pin(26, Pin.OUT)


# Set up the CO2 sensors
class MHZ19BSensor:

    # initializes a new instance
    def __init__(self, tx_pin, rx_pin, uartnum, co2_threshold):
        self.uart = UART(uartnum, baudrate=9600, bits=8, parity=None, stop=1, tx=Pin(tx_pin), rx=Pin(rx_pin))
        self.lights = 1
        self.co2_threshold = int(co2_threshold)

    def enable_auto_calibration(self):
        self.uart.write(b'\xff\x01\x79\xa0\x00\x00\x00\x00\xe6')
        time.sleep(1)
    
    def disable_auto_calibration(self):
        self.uart.write(b'\xff\x01\x79\0x00\x00\x00\x00\x00\xe6')
        time.sleep(1)
        
    # measure CO2
    def measure(self):
        while True:
            # send a read command to the sensor
            self.uart.write(b'\xff\x01\x86\x00\x00\x00\x00\x00\x79')

            # a little delay to let the sensor measure CO2 and send the data back
            time.sleep(.1)  # in seconds

            # read and validate the data
            buf = self.uart.read(9)
            if self.is_valid(buf):
                break

            # retry if the data is wrong

            print('error while reading MH-Z19B sensor: invalid data')
            print('retry ...')



        co2 = buf[2] * 256 + buf[3]
        return co2
        #print('co2         = %.2f' % co2)


    # check data returned by the sensor
    def is_valid(self, buf):
        if buf is None or buf[0] != 0xFF or buf[1] != 0x86:
            return False
        i = 1
        checksum = 0x00
        while i < 8:
            checksum += buf[i] % 256
            i += 1
        checksum = ~checksum & 0xFF
        checksum += 1
        return checksum == buf[8]
    
    
mySensor1 = MHZ19BSensor(0, 1, 0, 5)
mySensor2 = MHZ19BSensor(4, 5, 1, 5)

# Disable auto calibration
mySensor1.disable_auto_calibration()
mySensor2.disable_auto_calibration()


def push_screen(line1, line2):
    lcd.clear()
    lcd.move_to(0,0)
    lcd.putstr(line1+"\n")
    lcd.putstr(line2)


def boot():
    backlight_on()
    lcd.putstr('System Booting..')
    lcd_message_toggle = 0
    lcd_tick_count = 0
    for i in range(MHZ19B_warmupsecs, 0, -1):
        if lcd_tick_count >= 5:
            lcd_message_toggle = 1 - lcd_message_toggle
            lcd_tick_count = 0
        if lcd_message_toggle == 0:
            push_screen('System Booting..', 'Wait '+str(i)+' secs')
        else:
            push_screen('CO2 Sens.Warming', 'Wait '+str(i)+' secs')
        lcd_tick_count += 1
        
        time.sleep(1)
    
def clock_ended(start):
    print("time: ", time.ticks_ms() - start)
    if time.ticks_ms() - start > 1000:
        return True
    else:
        return False    
    
boot()    

def get_sensor_readings():
    pass

# write me a function that will return the elapsed time in the following format: xd xh xm xs
def pretty_time(current_time):
    return str(int(current_time/86400)) + 'd' + str(int((current_time%86400)/3600)) + 'h' + str(int(((current_time%86400)%3600)/60)) + 'm' + str(int(((current_time%86400)%3600)%60)) + 's'

global_time = 0
start_tick = time.ticks_ms()

while True:
    backlight_on()
    current_time = (time.ticks_ms() - start_tick) / 1000
    control_co2, experiment_co2 = mySensor1.measure(), mySensor2.measure()
    control_hpa, experiment_hpa, plant_hpa = get_pres_hPa(bmp1, 25), get_pres_hPa(bmp2, 25), get_pres_hPa(bmp3, 25)
    control_C, experiment_C, plant_C = get_tempC(bmp1, 25), get_tempC(bmp2, 25), get_tempC(bmp3, 25)

    push_screen('C:' + str(control_co2)+'ppm ' + str(round(control_C, 2)) +'C', 'E:' + str(experiment_co2)+'ppm ' + str(round(experiment_C, 2)) + 'C')
    time.sleep(2.5)
    
    push_screen('T: '+ pretty_time(current_time), 'Set CO2:' + str(CO2_threshold) + 'ppm')
    time.sleep(2.5)
    
    push_screen('hPa: ' + str(round(plant_hpa, 2)), str(round(control_hpa, 2)) + ' ' + str(round(experiment_hpa, 2)))
    time.sleep(2.5)
    
    push_screen('C:' + str(round(plant_C, 2)), str(round(control_C, 2)) + ' ' + str(round(experiment_C, 2)))
    time.sleep(2.5)
    
    

# Wiring for the project:

# Sensor 1: BMP180
# VIN: 3.3V
# GND: GND
# SCL: GP8
# SDA: GP9

# Sensor 2: BMP180
# VIN: 3.3V
# GND: GND
# SCL: GP2
# SDA: GP3

# Sensor 3: BMP180
# VIN: 3.3V
# GND: GND
# SCL: GP6
# SDA: GP7



# Sensor Pinout
# Wire 0: Yellow: NC
# Wire 1: Green: TX
# Wire 2: Blue: RX
# Wire 3: Red: VCC
# Wire 4: Black: GND
# Wire 5: White: NC
# Wire 6: Brown: NC

# Sensor 1: MH-Z19B:
# UART 0
# TX: GP0 (Blue Wire)
# RX: GP1 (Green Wire)

# Sensor 2: MH-Z19B:
# UART 1
# TX: GP4 (Blue Wire)
# RX: GP5 (Green Wire)