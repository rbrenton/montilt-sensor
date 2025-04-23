# MonTilt Sensor

MonTilt Sensor is ESP32 firmware that detects physical orientation using an MPU-6050 accelerometer and sends this data to the [MonTilt Host](https://github.com/rbrenton/montilt-host) application. This is part of the MonTilt system for automatic monitor rotation.

## Features

- Detects four orientations (0°, 90°, 180°, 270°) using an MPU-6050 accelerometer
- Communicates with the host application via USB serial connection
- Sends orientation data in JSON format
- Includes device MAC address for automatic identification
- Auto-detection by the host application

## Hardware Requirements

- ESP32 development board (ESP32 DOIT DevKit V1 recommended)
- MPU-6050 accelerometer module
- Jumper wires

## Wiring

Connect the MPU-6050 to the ESP32:
- VCC → 3.3V
- GND → GND
- SCL → GPIO22
- SDA → GPIO21

## Building and Flashing

This project uses PlatformIO:

1. Install [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html) or [PlatformIO IDE](https://platformio.org/install/ide)
2. Clone this repository
3. Build and upload:
   ```
   platformio run --target upload
   ```

## Protocol

The sensor communicates with the host via serial (115200 baud) using JSON messages:

```json
{
  \"mac\": \"XX:XX:XX:XX:XX:XX\",
  \"orientation\": 0,
  \"x\": 0.98,
  \"y\": 0.02,
  \"z\": 0.15
}
```

Where orientation is:
- 0: Landscape (0°)
- 1: Portrait - Right (90°)
- 2: Landscape - Upside Down (180°)
- 3: Portrait - Left (270°)

## Related Projects

- [MonTilt Host](https://github.com/rbrenton/montilt-host) - Windows application for monitor rotation management