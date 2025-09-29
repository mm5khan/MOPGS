# Multifunctional Organic Plant Growth System (Raspberry Pi + Arduino Nano) 
A Raspberry Pi–based automation script that monitors and controls a small plant-growth setup. It reads sensors (light, soil moisture, pH, tank levels, temperature/humidity), drives actuators (water pump, fertilizer pump, solenoid valve, UV/LED light, AC/cooling), displays status on a 20×4 LCD, and syncs everything with a Blynk IoT dashboard. It can also send an SMS status report via a GSM module.

## ✨ Features
- Auto/Manual modes via Blynk virtual pins
- Live telemetry: temperature, humidity, pH (scaled), moisture, light, water/fertilizer tank levels
- Actuator control: water pump, fertilizer pump, valve, light, AC
- 20×4 LCD status display
- SMS status using AT commands (GSM module on UART)
- I²C link to Arduino Nano for analog sensor aggregation
- Ultrasonic level measurement for water/fertilizer tanks

## 🧰 Hardware
- Raspberry Pi (with GPIO, I²C, UART enabled)
- Arduino Nano (I²C slave at 0x48, sends light;moist;ph as ASCII)
- DHT11 temperature/humidity sensor (on GPIO4 / board.D4)
- 20×4 LCD (HD44780) in 4-bit mode
- Relays (light, water pump, fertilizer pump, valve, AC)
- 2× HC-SR04 (or similar) ultrasonic sensors for tank levels
- GSM module (UART on /dev/ttyS0)
- Assorted wiring, 5V supply for logic and relay boards
> ⚠️ Safety: Pumps/AC/UV/light may involve mains voltage or high current. Use proper isolation (relay modules/SSR), fuses, and enclosures. Double-check wiring before powering.

## Wiring (BCM numbering)
| Function            | GPIO (BCM) |
|:-------------------:|:----------:|
|Light Relay | 5|
Water Pump Relay	13
Fertilizer Pump Relay	19
Solenoid Valve Relay	6
AC/Cooling Relay	26
Water Level TRIG (Tx)	21
Water Level ECHO (Rx)	20
Fertilizer Level TRIG	16
Fertilizer Level ECHO	12
LCD RS	7
LCD EN	8
LCD D4..D7	25,24,23,18
DHT11 Data	4 (board.D4)
I²C to Arduino	SDA, SCL (default bus 1)
GSM Module	UART /dev/ttyS0 (Tx/Rx)
