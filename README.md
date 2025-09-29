# Multifunctional Organic Plant Growth System (Raspberry Pi + Arduino Nano) 
A Python application that orchestrates a system for plant care. It’s designed like a small backend: event-driven callbacks (Blynk), a deterministic control loop (scheduler), typed boundaries for I/O (GPIO/I²C/UART adapters), and a clear contract between the UI (Blynk virtual pins) and the control domain. This README highlights the software architecture, API surface, testing strategy, observability, and extensibility—not the wiring.

## Features
- Exposes a virtual-pin API (Blynk) for mode/actuator control and telemetry streaming
- Pulls sensor frames from an I²C bus (Arduino acts as a data provider)
- Runs a policy-driven controller that translates sensor inputs → actuator intents
- Publishes device state to a real-time dashboard and an LCD view
- Ships a timer wheel (BlynkTimer) to schedule periodic jobs with backoff-friendly cycles
- Provides an SMS transport (UART AT commands) as a pluggable notification channel

## Hardware
- Raspberry Pi (with GPIO, I²C, UART enabled)
- Arduino Nano (I²C slave at 0x48, sends light;moist;ph as ASCII)
- DHT11 temperature/humidity sensor (on GPIO4 / board.D4)
- 20×4 LCD (HD44780) in 4-bit mode
- Relays (light, water pump, fertilizer pump, valve, AC)
- 2× HC-SR04 (or similar) ultrasonic sensors for tank levels
- GSM module (UART on /dev/ttyS0)
- Assorted wiring, 5V supply for logic and relay boards
> ⚠️ Safety: Pumps/AC/UV/light may involve mains voltage or high current. Use proper isolation (relay modules/SSR), fuses, and enclosures. Double-check wiring before powering.

## Architecture
- `run.py` (this script): composition root; wires adapters, timers, and handlers.
- Adapters (edge I/O):
  * `BlynkAdapter`: maps virtual pins ⇄ domain commands/events
  * `I2CAdapter`: pulls a sensor frame (`light;moist;ph`) → typed DTO
  * `GpioAdapter`: applies actuator intents → GPIO pins (idempotent writes)
  * `UartSmsAdapter`: `send(message)` via AT commands (transport boundary)
  * `DhtAdapter`: reads temp/humidity (single-sensor adapter)
  * `LcdView`: dumb view that renders current snapshot
- Domain:
  *`Controller`: pure(ish) policy: thresholds in → `ActuatorIntent`
  *`State`: source of truth for inputs, derived values, outputs
  *`Schedulers`: periodic tasks: `pollSensors`, `reconcile`, `publish`, `render`

## Data flow (pull → decide → apply → publish)
[I2C + DHT] --> SensorFrame  ┐
                             |--> Controller(policy) --> ActuatorIntent --> GPIO
[Blynk V-pins] --> Commands  ┘                                  |
                                                             [State]
                              ┌------------------------------┴----------------------┐
                              |              Publishers (views/transports)          |
                              |   Blynk telemetry    LCD view     SMS (on demand)  |
                              └----------------------------------------------------┘
## Execution Model
- Event-driven: Blynk virtual pin callbacks (manual commands)
- Time-driven: `BlynkTimer` schedules (2s/3s/4s) for poll/apply/publish
- Single-threaded cooperative scheduling (no shared-state races)

## Run Modes
### Production (real hardware):
```
sudo -E python3 app.py
```

### Mock mode (no hardware; great for demos/CI):
```
MOCK_MODE=true python3 app.py
```
- I²C → returns deterministic or random but bounded values
- GPIO → logs intents instead of toggling pins
- UART → prints SMS payload to stdout
- DHT → synthetic temp/humidity
- LCD → ASCII render to stdout
