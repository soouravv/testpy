import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from time import sleep
import adafruit_dht
import RPi.GPIO as GPIO

# Define pin for LDR
LDR_PIN = 27

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(LDR_PIN, GPIO.IN)

# Initialize I2C bus and ADC
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, address=0x48)

# Define analog input channel for rain sensor
RAIN_SENSOR_CHANNEL = 0
rain_sensor = AnalogIn(ads, RAIN_SENSOR_CHANNEL)

# Initialize DHT11 sensor
dht11 = adafruit_dht.DHT11(board.D17)

# Define LCD constants
LCD_WIDTH = 16
LCD_LINES = 2
LCD_ADDRESS = 0x27
LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_BACKLIGHT = 0x08
E_PULSE = 0.0005
E_DELAY = 0.0005
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02

# Function to initialize LCD
def lcd_init():
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    sleep(E_DELAY)

# Function to write byte to LCD
def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(LCD_ADDRESS, bits_high)
    lcd_toggle_enable(bits_high)
    bus.write_byte(LCD_ADDRESS, bits_low)
    lcd_toggle_enable(bits_low)

# Function to toggle LCD enable
def lcd_toggle_enable(bits):
    sleep(E_DELAY)
    bus.write_byte(LCD_ADDRESS, (bits | 0x04))
    sleep(E_PULSE)
    bus.write_byte(LCD_ADDRESS, (bits & ~0x04))
    sleep(E_DELAY)

# Function to clear LCD
def lcd_clear():
    lcd_byte(LCD_CLEARDISPLAY, LCD_CMD)
    lcd_byte(LCD_RETURNHOME, LCD_CMD)
    sleep(E_DELAY)

# Function to display string on LCD
def lcd_string(message, line):
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]), LCD_CHR)

# Initialize LCD
lcd_init()
lcd_clear()
lcd_string("System Loading", LCD_LINE_1)
sleep(0.5)

# Main loop
while True:
    try:
        # Read temperature and humidity from DHT11
        temperature = dht11.temperature
        humidity = dht11.humidity

        # Read analog input from rain sensor
        rain_sensor_value = rain_sensor.value

        # Display temperature, humidity, and rain sensor value on LCD
        lcd_clear()
        lcd_string("Temp: {:.1f}C".format(temperature), LCD_LINE_1)
        lcd_string("Humidity: {}%".format(humidity), LCD_LINE_2)
        sleep(2)
        lcd_clear()
        lcd_string("Rain Sensor: {}".format(rain_sensor_value), LCD_LINE_1)
        sleep(2)

        # Print values to console
        print("Temperature: {:.1f}C, Humidity: {}%, Rain Sensor: {}".format(
            temperature, humidity, rain_sensor_value))

    except Exception as e:
        print("Error:", e)

    sleep(1)
