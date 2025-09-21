from tkinter import Variable #importing variable from tkinter
import board  # Package reference RPi pins enumeration (Used for Pin 4 of RPi for the temperature sensor)
import smbus  # Package referenced to use system management bus for the I2C protocol (Used for the connection between RPI and Arduino Nano)
import time  # To used time sleep in the code
import RPi.GPIO as GPIO  # Used to reference the GPIOs pins of RPI and related functions
import adafruit_dht  # Temperature sensor package
import serial  # Used to open a serial port to communication with the GSM Module via the Tx/Rx pins on RPI
from RPLCD import (
    CharLCD,
)  # Package used to define LCD pins and control information being displayed on it
import BlynkLib  # Blynk IoT Package used to refere to different functions of the IoT Cloud service
from BlynkTimer import (
    BlynkTimer,
)  # Package used to set a server based timer for the blynk to run each of the system functions on a blynk timer.
import random  # Package used to generate random values (Used for testing only)

# from calendar import c

# Blynk library Authentication code
BLYNK_AUTH = ""

# Connect to blynk server using blynklin package functions
blynk = BlynkLib.Blynk(BLYNK_AUTH, server="blynk.cloud")

# Disable GPIO warning messages (Appears sometime when trying to set a GPIO pin as IN/OUT while it's status already set)
GPIO.setwarnings(False)

# Define the LCD pins using the package CharLCD functions (GPIO.BCM 'Broadcom SOC channel' numbering convention being used to select pins)
lcd = CharLCD(pin_rs=7, pin_e=8, pins_data=[25, 24, 23, 18], numbering_mode=GPIO.BCM)

# Hiding the cursor blinking on the LCD
lcd.cursor_mode = "hide"

# Selecting the I2C address and enabling the communication channel (The Address should Match the one used in Arduino Nano uploaded code)
i2c_address = 0x48
bus = smbus.SMBus(1)

# Defining and initializing several variables to be used in the code below
auto = 0
data = ""
lightLevel = 0
mositureLevel = 0
phLevel = 0

# Defining an object for the Temperature Sensor using adafruit_dht package
dhtDevice = adafruit_dht.DHT11(board.D4, use_pulseio=False)

# Synchronizing the blynk IoT virtual pins by mean reflecting the current server status on the real prototype status.
blynk.sync_virtual(2, 3, 7, 9, 12, 13)

# Define new Blynk time object
timer = BlynkTimer()

# Define new serial port object for to communicate with the GSM Module
ser = serial.Serial("/dev/ttyS0", 115200, timeout=12)

# Delay for 2 Seconds used to insure that above objects definitions and blynk server synchronization has been done
time.sleep(2)

# Define pin numbers to control relays and ultrasonic sensors
# (For ultrasonic sensors Tx = Trig, Rx = Echo)
# Note that the BCM number convention has been used and below is the full layout of the RPI connection for more clarity

#                    3.3     ----   5V  <--- Relays Switching Vcc
#   Arduino A4  ---> SDA     ----   5V  <--- LCD Power On
#   Arduino A5  ---> SCL     ----   GND <--- GSM Module GND & Relays Switching GND
#   Temp Sensor ---> GPIO4   ----   Tx  <--- GSM Module Rx
#                    GND     ----   Rx
#                    GPIO17  ----   GPIO18 <--- LCD Data 7
#                    GPIO27  ----   GND
#                    GPIO22  ----   GPIO23 <--- LCD Data 6
#                    3.3v    ----   GPIO24 <--- LCD Data 5
#                    GPIO10  ----   GND
#                    GPIO09  ----   GPIO25 <--- LCD Data 4
#                    GPIO11  ----   GPIO08 <--- LCD EN
#                    GND     ----   GPIO07 <--- LCD RS
#                    ID_SD   ----   ID_SD
#   UV Light    ---> GPIO05  ----   GND
#   Valve       ---> GPIO06  ----   GPIO12 <--- Fertilizer Ultrasonic Echo
#   Water       ---> GPIO013 ----   GND
#   Fertilizer  ---> GPIO019 ----   GPIO16 <--- Fertilizer Ultrasonic Trig
#   Ac          ---> GPIO026 ----   GPIO20 <--- Water Ultrasonic Echo
#                    GND     ----   GPIO21 <--- Water Ultrasonic Trig

lightPin = 5
waterPumpPin = 13
fertiliserPumpPin = 19
valvePin = 6
acPin = 26
waterTxPin = 21
fertiliserTxPin = 16
waterRxPin = 20
fertiliserRxPin = 12

# Define GPIOs as input/output accordingly
GPIO.setup(lightPin, GPIO.OUT)
GPIO.setup(waterPumpPin, GPIO.OUT)
GPIO.setup(fertiliserPumpPin, GPIO.OUT)
GPIO.setup(valvePin, GPIO.OUT)
GPIO.setup(acPin, GPIO.OUT)
GPIO.setup(waterTxPin, GPIO.OUT)
GPIO.setup(fertiliserTxPin, GPIO.OUT)
GPIO.setup(waterRxPin, GPIO.IN)
GPIO.setup(fertiliserRxPin, GPIO.IN)
time.sleep(4)

# Initialize output pins to low level
GPIO.output(lightPin, 0)
GPIO.output(waterPumpPin, 0)
GPIO.output(fertiliserPumpPin, 0)
time.sleep(1)
GPIO.output(valvePin, 0)
GPIO.output(acPin, 0)

# Configurable Thresholds for light, moisture, fertilizer and temp levels
lightThrshold = 0.5  # 0 - 2.5
mositureThreshold = 3  # 0 - 5
fertiliserThreshold = 3  # 0 - 5
tempThreshold = 25  # Temp C

# Initialize Variables to be used for displaying data on the LCD
# and SMS Status message
waterPump = 0
most = 0
fertiliserPump = 0
phLevel = 0
light = 0
lightLevel = 0
valve = 0
fertiliserLevel = 0
temp = 0
humidity = 0
ac = 0

####################### Blynk Server Functions #########################
# Functions used for receiving commands from the blynk server


@blynk.VIRTUAL_WRITE(12)
def my_write_handler(value):
    """Activated when the Auto Button is pressed to set Auto mode ON/OFF"""
    global auto
    auto = value
    print(auto)


@blynk.VIRTUAL_WRITE(13)
def my_write_handler(value):
    """Activated when the AC Button is pressed to set AC ON/OFF Manually when Auto is OFF"""
    global ac
    if auto == ["0"]:
        if value == ["1"]:
            GPIO.output(acPin, 1)
            ac = 1
        else:
            ac = 0
            GPIO.output(acPin, 0)


@blynk.VIRTUAL_WRITE(9)
def my_write_handler(value):
    """Activated when the Valve Button is pressed to set Valve ON/OFF Manually when Auto is OFF"""
    global valve
    if auto == ["0"]:
        if value == ["1"]:
            valve = 1
            GPIO.output(valvePin, 1)
        else:
            valve = 0
            GPIO.output(valvePin, 0)


@blynk.VIRTUAL_WRITE(2)
def my_write_handler(value):
    """Activated when the Water Pump Button is pressed to set the pump ON/OFF Manually when Auto is OFF"""
    global waterPump
    if auto == ["0"]:
        if value == ["1"]:
            waterPump = 1
            GPIO.output(waterPumpPin, 1)
        else:
            waterPump = 0
            GPIO.output(waterPumpPin, 0)


#
@blynk.VIRTUAL_WRITE(3)
def my_write_handler(value):
    """Activated when the Fertilizer Pump Button is pressed to set the pump ON/OFF Manually when Auto is OFF"""
    global fertiliserPump
    if auto == ["0"]:
        if value == ["1"]:
            fertiliserPump = 1
            GPIO.output(fertiliserPumpPin, 1)
        else:
            fertiliserPump = 0
            GPIO.output(fertiliserPumpPin, 0)


@blynk.VIRTUAL_WRITE(7)
def my_write_handler(value):
    """Activated when the Light Button is pressed to set the light ON/OFF Manually when Auto is OFF"""
    global light
    if auto == ["0"]:
        if value == ["1"]:
            light = 1
            GPIO.output(lightPin, 1)
        else:
            light = 0
            GPIO.output(lightPin, 0)


@blynk.VIRTUAL_WRITE(14)
def my_write_handler(value):
    """Activated when the SMS Button is pressed to Send System Status Message Manually when Auto is OFF"""
    if value == ["1"]:
        sendsms(
            "System Status:\n\nTemp:{:.1f}C Hum:{:.1f}%\nPH:{:.1f} MOST:{:.2f}%\nWLVL{:.1f}% FLVL{:.1f}%\nLHT{} AC{}\nWPMP{} FPMP{}".format(
                temp,
                humidity,
                phLevel,
                most,
                (waterLevel / 18) * 100,
                (fertiliserLevel / 18) * 100,
                light,
                ac,
                waterPump,
                fertiliserPump,
            )
        )


def readWaterLevel():
    """Read water container water level using an ultrasonic sensor distance measurement"""
    global waterLevel
    trig = waterTxPin
    echo = waterRxPin
    GPIO.output(trig, False)
    # Allow module to settle
    time.sleep(0.5)
    # Send 10us pulse to trigger
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    pulse_start = time.time()

    # save StartTime
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    # save time of arrival
    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    # time difference between start and arrival
    pulse_duration = pulse_end - pulse_start
    # mutiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = pulse_duration * 17150

    distance = round(distance, 2)

    # Subtracting 18 from the distance so when the container is full
    # then the value will be zero which will be reflected onto blynk dashboard water level widget as 0%
    waterLevel = 18 - distance
    if waterLevel < 0:
        waterLevel = 0


def readFertiliserLevel():
    """Read fertilizer container fertilizer level using an ultrasonic sensor distance measurement"""
    global fertiliserLevel
    trig = fertiliserTxPin
    echo = fertiliserRxPin
    GPIO.output(trig, False)
    # Allow module to settle
    time.sleep(0.5)
    # Send 10us pulse to trigger
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    pulse_start = time.time()

    # save StartTime
    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    # save time of arrival
    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    # time difference between start and arrival
    pulse_duration = pulse_end - pulse_start
    # mutiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = pulse_duration * 17150

    distance = round(distance, 2)

    # Subtracting 18 from the distance so when the container is full
    # then the value will be zero which will be reflected onto blynk dashboard water level widget as 0%
    fertiliserLevel = 18 - distance
    if fertiliserLevel < 0:
        fertiliserLevel = 0


def sendsms(message):
    """Function used to send an SMS message using AT Commands via the serial port object (ser)"""
    ser.write(b"AT\r")  # Enter command mode
    time.sleep(1)
    ser.write(b"AT+CMGF=1\r")  # Select the operation mode to 1 (Sending SMS Message)
    time.sleep(1)
    ser.write(
        b'AT+CMGS="+97333091895"\r'
    )  # Providing the Mobile Number the the SMS will be sent to
    time.sleep(1)
    ser.write(message.encode() + b"\r")  # message contans the system status
    time.sleep(1)
    ser.write(
        chr(26).encode()
    )  # Activate send command by sending char(26) which is CTRL+Z
    time.sleep(1)
    ser.flush()  # Remove any remaining bytes in the serial buffer to avoid messages overlapping


def sendsms():
    """Function used to send an SMS message using AT Commands via the serial port object (ser)"""
    message = "System Status:\n\nTemp:{:.1f}C Hum:{:.1f}%\nPH:{:.1f} MOST:{:.2f}%\nWLVL{:.1f}% FLVL{:.1f}%\nLHT{} AC{}\nWPMP{} FPMP{}".format(
        temp,
        humidity,
        phLevel,
        most,
        (waterLevel / 18) * 100,
        (fertiliserLevel / 18) * 100,
        light,
        ac,
        waterPump,
        fertiliserPump,
    )

    ser.write(b"AT\r")  # Enter command mode
    time.sleep(1)
    ser.write(b"AT+CMGF=1\r")  # Select the operation mode to 1 (Sending SMS Message)
    time.sleep(1)
    ser.write(
        b'AT+CMGS="+97333091895"\r'
    )  # Providing the Mobile Number the the SMS will be sent to
    time.sleep(1)
    ser.write(message.encode() + b"\r")  # message contains the system status
    time.sleep(1)
    ser.write(
        chr(26).encode()
    )  # Activate send command by sending char(26) which is CTRL+Z
    time.sleep(1)
    ser.flush()  # Remove any remaining bytes in the serial buffer to avoid messages overlapping


def controlDevices():
    """Functions used to change the status of relays and receive sensors data from the Arduino Nano via I2C protocol"""
    # Global variable means its value could be referred to outside of this function scope (reading its value only)
    global waterPump
    global most
    global fertiliserPump
    global phLevel
    global light
    global lightLevel
    global valve
    global fertiliserLevel
    global temp
    global humidity
    global ac
    global data

    # Check the data received from Arduino Nano which would be 14 characters as follows:
    # Light Sensor      ---> value between 0 - 1023
    # Moisture Sensor   ---> value between 0 - 1023
    # phLevel Sensor    ---> value between 0 - 1023
    # and between each value there will be a semi-colon saparator as below example
    # data = "0123;1023;0032"
    # Light = 123, Most = 1023, ph = 32
    # These values will be then converted into voltages to get the correct sensor measurements

    # Initialize data variable to be 0 for the 3 sensor values
    data = "0000;0000;0000"

    # Loop the data received from the Arduino Nano which as mentioned would be 14 characters
    for i in range(0, 13):
        data += chr(bus.read_byte(i2c_address))

    # Splitt the data received by the semi-colon delimiter to retrive sensor values
    # Additionally each value is converted to integer then converted into voltage levels where 5 is the power input voltage level
    # but note that the voltage level of the phLevel Sensor is 12v thus multiplied by 12 instead of 5 as shown below
    lightLevel = (int(data.split(";")[0]) * 5.0) / 1024.0
    mositureLevel = (int(data.split(";")[1]) * 5.0) / 1024.0
    phLevel = (int(data.split(";")[2]) * 12.0) / 1024.0

    # When the Auto Mode is active then check the sensors values againest predefined thresholds
    # To send the corrresponding device relay ON/OFF accordingly
    if auto == ["1"]:
        try:  # Try is used to catch runtime errors
            # Check the light level and switch ON/OFF the corresponding relay
            if lightLevel < lightThrshold:
                # print("Night Time!")
                light = 1
                GPIO.output(lightPin, 1)

            elif lightLevel >= lightThrshold:
                # print("Morning Time!")
                light = 0
                GPIO.output(lightPin, 0)

            # Check the moisture level and change waterpump variable to 1/0 accordingly
            if mositureLevel < mositureThreshold:
                # print("Wet Soil!")
                waterPump = 0

            elif mositureLevel >= mositureThreshold:
                # print("Dry Soil!")
                waterPump = 1

            # Check the ph level and change fertilizer variable to 1/0 accordingly
            fertiliserPump = 0
            if phLevel < fertiliserThreshold:
                # print("Fertilized Soil!")
                fertiliserPump = 0

            elif phLevel >= fertiliserThreshold:
                # print("Non Fertilized Soil!")
                fertiliserPump = 1

            # Change the valve value to 1/0 if waterpump or fertilizerpump variables are 1 and if neither is then sent the valve to 0
            valve = 1 if waterPump or fertiliserPump else 0

            # Change the valve, waterpump, fertilzerpump relays to ON/OFF according to the previous related if statements output
            GPIO.output(valvePin, valve)
            GPIO.output(waterPumpPin, waterPump)
            GPIO.output(fertiliserPumpPin, fertiliserPump)

        except:
            pass  # If error then do nothing
    try:
        # Get the temperature and humidity data from the DHT sensor which is connected to GPIO.D4
        temp = dhtDevice.temperature
        humidity = dhtDevice.humidity

        # Check if the temp and humidity values is measured then if the temp is less than the threshold temp
        # then switch OFF the AC relay and if it is above threshold then switch AC to ON
        if temp and humidity:
            if temp < tempThreshold:
                if auto == ["1"]:
                    ac = 0
                    GPIO.output(acPin, 0)

            elif temp >= tempThreshold:
                if auto == ["1"]:
                    ac = 1
                    GPIO.output(acPin, 1)
    except RuntimeError as error:
        pass

    # phLevel = (phLevel / 9840) * 10         # Will return values between 0 - 10 from the pH Sensor
    # phLevel = random.uniform(7, 7.6)

    # Store the mositure and light levels to be used with the blynk
    most = mositureLevel


def controlBlynk():
    """This function will update the status of the system in the blynk IoT dashboard by refering to each of defined virtual pin"""
    # If the Auto mode is active then control water Pump, fertilizer Pump, light and valve according to
    # the sensor data automatically
    if auto == ["1"]:
        blynk.virtual_write(2, waterPump)
        blynk.virtual_write(3, fertiliserPump)
        blynk.virtual_write(7, light)
        blynk.virtual_write(9, valve)
        if not ac == None:
            blynk.virtual_write(13, ac)

    blynk.virtual_write(0, phLevel)
    blynk.virtual_write(1, mositureLevel)
    blynk.virtual_write(8, lightLevel)
    if not waterLevel == None:
        blynk.virtual_write(5, waterLevel)
    if not fertiliserLevel == None:
        blynk.virtual_write(6, fertiliserLevel)

    # lynk.virtual_write(11, data)
    if not humidity == None:
        blynk.virtual_write(4, humidity)
    if not temp == None:
        blynk.virtual_write(10, temp)


def lcd_data():
    """This function will update the data displayed on the LCD"""
    lcd.clear()
    time.sleep(0.1)
    lcd.cursor_pos = (0, 0)  # (Rows, Columns)
    if not temp == None and not humidity == None:
        lcd.write_string("Temp:{:.1f}C Hum:{:.1f}%".format(temp, humidity))

    lcd.cursor_pos = (1, 0)
    time.sleep(0.1)
    lcd.write_string("PH:{:.1f} MOST:{:.2f}%".format(phLevel, most))
    lcd.cursor_pos = (2, 0)
    time.sleep(0.1)
    lcd.write_string(
        "WLVL{:.1f}% FLVL{:.1f}%".format(
            (waterLevel / 18) * 100, (fertiliserLevel / 18) * 100
        )
    )
    lcd.cursor_pos = (3, 0)
    time.sleep(0.1)
    lcd.write_string(
        "LHT{} AC{} WPMP{} FPMP{}".format(light, ac, waterPump, fertiliserPump)
    )


# The below are list of blynk timers which will call the above functions
# in differnet time intervales, and the menioned numbers are intervals in seconds
# Note that the set_timeout will run once, while set_interval will keep running every x seconds
timer.set_timeout(1, readWaterLevel)
timer.set_timeout(1, readFertiliserLevel)
timer.set_timeout(1, controlDevices)
timer.set_interval(2, controlDevices)
timer.set_interval(2, controlBlynk)
timer.set_interval(4, readWaterLevel)
timer.set_interval(4, readFertiliserLevel)
timer.set_interval(3, lcd_data)

# While loop to excute the blynk and blynk timer and keep the connection alive.
while True:
    blynk.run()
    timer.run()
