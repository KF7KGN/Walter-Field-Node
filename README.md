# Walter Field Node
## LonePeak Radio LLC -- ESP32-S3 Cellular IoT Sensor Platform

Production firmware for the Walter Field Node -- a solar/battery-powered
remote sensor platform built on the DPTechnics Walter (ESP32-S3 + Sequans
GM02SP cellular modem). Deployed in the field monitoring a trailer in Utah.

## What It Does

- Reads DS18B20 temperature probe on GPIO12
- Reads 0-25V trailer battery voltage on GPIO4
- Reads BME680 environmental sensor (temp, humidity, pressure, gas, air quality)
- Sends all readings as JSON telemetry via UDP every 15 minutes
- Primary transport: LTE cellular via Soracom
- Failover transport: WiFi UDP to same relay server with identical payload
- Serves a local captive portal dashboard over WiFi AP with auto-refresh
- Login-gated dashboard showing signal bars, sensor readings, uplink stats

## Hardware

- MCU: DPTechnics Walter (ESP32-S3 + Sequans GM02SP cellular modem)
- Temperature: DS18B20 waterproof probe on GPIO12
- Battery monitor: 0-25V sensor module on GPIO4
- Environment: BME680 on I2C (SDA=GPIO8, SCL=GPIO9, addr 0x76)
- Cellular: LTE-M / NB-IoT via Soracom
- Power: Solar + 12V trailer battery

## Wiring

DS18B20:
  Red    -> 3V3
  Black  -> GND
  Yellow -> GPIO12
  4.7k resistor between Yellow and Red (REQUIRED)

0-25V Battery Sensor:
  S (signal) -> GPIO4
  VCC        -> 3V3
  GND        -> GND
  Screw+     -> Battery positive
  Screw-     -> Battery negative

BME680:
  VCC -> 3V3
  GND -> GND
  SDA -> GPIO8
  SCL -> GPIO9
  CS  -> 3V3  (selects I2C mode)
  SDO -> GND  (selects address 0x76)

## Telemetry JSON Fields

  tc     - ESP32 CPU silicon temperature C
  ds     - DS18B20 probe temperature C
  vb     - Trailer battery voltage V
  bme_t  - BME680 temperature C
  bme_h  - BME680 humidity percent RH
  bme_p  - BME680 pressure hPa
  bme_g  - BME680 gas resistance ohms
  rsrp   - LTE signal power dBm
  rssi   - LTE signal strength dBm
  up     - Uptime seconds since boot
  op     - Cellular operator name
  seq    - Transmission sequence number

## Setup

1. Install Arduino IDE
2. Install ESP32 board package
3. Install libraries via Arduino Library Manager:
   - OneWire by Paul Stoffregen
   - DallasTemperature by Miles Burton
   - Adafruit BME680
   - Adafruit Unified Sensor
   - WalterModem by DPTechnics
4. Copy src/config.h.example to src/config.h
5. Fill in your WiFi credentials, APN, and relay server details
6. Flash to Walter via USB

NOTE: config.h is excluded from git.


## Changelog

v1.12 - WiFi telemetry failover. Cellular primary, WiFi UDP fallback.
        New debug counters txCell/txWifi/txFail and uplink status on dashboard.
v1.11 - BME680 environmental sensor on I2C. Non-blocking state machine.
        New JSON fields bme_t, bme_h, bme_p, bme_g. New dashboard card.
v1.10 - 0-25V trailer battery sensor on GPIO4. New vb field in JSON.
v1.09 - DS18B20 temperature probe on GPIO12. Non-blocking 12-bit reads.
v1.08 - UDP telemetry upload to Linode relay server.
v1.07b - setRAT AUTO to clear stuck NB-IoT.

## License

GNU General Public License v3.0
