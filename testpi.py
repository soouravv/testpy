from time import sleep
import board
import adafruit_dht
from smbus import SMBus
import RPi.GPIO as GPIO

LDR = 27
bus = SMBus(1)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set the sensor pin as Input pin
GPIO.setup(LDR, GPIO.IN)

# I2C address of the LCD
LCD_ADDRESS = 0x27
# Define some device constants
LCD_WIDTH = 16  # Maximum characters per line

# Define some device constants
LCD_CHR = 1  # Mode - Sending data
LCD_CMD = 0  # Mode - Sending command

# Define some device constants
LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line

# Define device parameters
LCD_BACKLIGHT = 0x08  # On
# LCD_BACKLIGHT = 0x00  # Off

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

# LCD commands
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT = 0x10
LCD_FUNCTIONSET = 0x20
LCD_SETCGRAMADDR = 0x40
LCD_SETDDRAMADDR = 0x80

# Function to initialize the LCD
def lcd_init():
    # Initialise display
    lcd_byte(0x33, LCD_CMD)  # 110011 Initialise
    lcd_byte(0x32, LCD_CMD)  # 110010 Initialise
    lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
    lcd_byte(0x0C, LCD_CMD)  # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
    lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
    sleep(E_DELAY)


# Function to write to the LCD
def lcd_byte(bits, mode):
    # Send byte to data pins
    # bits = the data
    # mode = 1 for data, 0 for command
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

    # High bits
    bus.write_byte(LCD_ADDRESS, bits_high)
    lcd_toggle_enable(bits_high)

    # Low bits
    bus.write_byte(LCD_ADDRESS, bits_low)
    lcd_toggle_enable(bits_low)


def lcd_toggle_enable(bits):
    # Toggle enable
    sleep(E_DELAY)
    bus.write_byte(LCD_ADDRESS, (bits | 0x04))
    sleep(E_PULSE)
    bus.write_byte(LCD_ADDRESS, (bits & ~0x04))
    sleep(E_DELAY)


# Function to clear the LCD
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


# Starting text
lcd_init()
lcd_string("System Loading", LCD_LINE_1)
for a in range(0, 16):
    lcd_string(".", LCD_LINE_2 + a)
    sleep(0.1)
lcd_clear()
lcd_string("Wait..", LCD_LINE_1)


# Create a object for the DHT11 sensor
DHT11 = adafruit_dht.DHT11(board.D17, use_pulseio=False)


def TempHumi():
    try:
        # Get the Temperature and Humidity values
        temperature_c = DHT11.temperature
        temperature_f = temperature_c * (9 / 5) + 32
        humidity = DHT11.humidity

        # Print the values on the LCD display
        lcd_string("T:" + str(temperature_c) + ".0C", LCD_LINE_1)
        lcd_string("H:" + str(humidity) + "%", LCD_LINE_2)

        # Print the values to the serial port
        print(
            "Temp: {:.1f} F / {:.1f} C    Humidity: {}% ".format(
                temperature_f, temperature_c, humidity
            )
        )

    except RuntimeError as error:
        # Errors happen fairly often, DHT's are hard to read, just keep going
        print(error.args[0])
        sleep(1)
    except Exception as error:
        DHT11.exit()
        raise error

    sleep(1)


def light():
    value = GPIO.input(LDR)
    if value == 0:
        lcd_string("L:" + "High", LCD_LINE_1 + 8)
    else:
        lcd_string("L:" + "LOW ", LCD_LINE_1 + 8)


# Get the analog input values
def rain():
    bus.write_byte(0x4b, 0x84)  # A0
    value = bus.read_byte(0x4b)
    value = (value / 255) * 100
    value = (value - 100) * -1

    value = int(value)
    lcd_string("R:" + str(value) + "% ", LCD_LINE_2 + 8)


while True:
    TempHumi()
    light()
    rain()
