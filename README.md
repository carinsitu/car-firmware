# Car firmware aka CarNode

This firmware is designed to be embedded in _Car In Situ_ RC car.

Although the main goal is to use _CarNode_ with a camera, this one is optionnal.

_CarNode_ allows to use a _Wemos D1 mini_ board, or similar, to turn an RC car into a WiFi-driveable one with this GPL licenced firmware.

Note: A fully featured dedicated board exists for this project to easily convert a _WLtoys K989_ RC car.

## Features

 * Discoverable over MDNS
 * Discover and automatically connect to _CarInSitu_ server
 * Send flow commands (ie. steering and throttle) through UDP
 * Send others commands through TCP
 * Drive PPM servos
 * Talk with VTX to setup embedded camera (e.g. Video channels)
 * Over-The-Air firmware uploads
 * Decode IR codes and send them to the server

## Wiring

Pin assignments match `src/CarBoard.cpp` and the default PlatformIO environment `pcb_v0_5` in `platformio.ini`.

Hardware UART0 is remapped with `Serial.swap()` so its pins appear on **D7/D8** instead of the **RX/TX** header. VTX SmartAudio uses **D8** as TX; the firmware inverts that UART TX line (`U0C0 |= BIT(UCTXI)`).

| Wemos D1 mini Pro GPIO | ESP8266 GPIO | Connected to | Notes |
|------------------------|--------------|--------------|-------|
| TX | 1 | Debug serial TX | `SoftwareSerial` @ 76800 baud |
| RX | 3 | Debug serial RX | `SoftwareSerial` @ 76800 baud |
| D0 | 16 | Front headlights | PWM (`analogWrite`); /!\ cannot wake from deep sleep |
| D1 | 5 | I²C devices¹ | I²C SCL (Arduino `Wire` default) |
| D2 | 4 | I²C devices¹ | I²C SDA (Arduino `Wire` default) |
| D3 | 0 | Steering servo | PPM; /!\ boot mode (1: run / 0: flash) |
| D4 | 2 | Rear RGB LED (WS2812B)⁰ | NeoPixel via UART1 TX; /!\ must be high at boot |
| D5 | 14 | Throttle ESC | PPM |
| D6 | 12 | IR receiver | TSOP3xx series, RC6 decoded |
| D7 | 13 | Hardware UART0 RX (after `Serial.swap`) | Unused by firmware |
| D8 | 15 | VTX SmartAudio | Hardware UART0 TX after `Serial.swap`, signal inverted; /!\ must be low at boot |
| A0 | ADC0 | Battery voltage | Raw `analogRead`; see also MAX17261 on I²C |

 - [0] WS2812B RGB LED behind a level shifter (LVC1T45); driven through UART1 by NeoPixelBus (`NeoEsp8266Uart1800KbpsMethod`), single pixel (`length = 1` in `src/CarBoard.cpp`).
 - [1] I²C devices on D1 (SCL) / D2 (SDA):
   * Battery gauge (MAX17261) — Monitors battery voltage and state of charge
   * IMU (LSM6DS3) — 6-axis inertial measurement unit with accelerometer and gyroscope

## Development environment

### Setup platformio

```shell
pip install --user virtualenv
```

Add in `~/.profile`:

```shell
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi

if [ -d "$HOME/.platformio/penv/bin" ] ; then
    PATH="$HOME/.platformio/penv/bin:$PATH"
fi
```

Reload `~/.profile`:

```shell
source ~/.profile
```

```shell
virtualenv $HOME/.platformio/penv
```

### Usage

 * Compile

     ```
     pio run
     ```

 * Compile and upload through serial

    ```
    pio run -t upload
    ```

 * Compile and upload through OTA

    ```
    avahi-browse -a -t # List available mDNS services
    avahi-browse -t _arduino._tcp | grep CarNode | awk -F' ' '{ print $4 }' # List car nodes mDNS hostnames
    pio run --target upload --upload-port TARGET_FQDN.local
    ```

    If you want a massive OTA update:

    ```
    for i in `avahi-browse -t _arduino._tcp | grep CarNode | awk -F' ' '{ print $4 }'`; do
      pio run --target upload --upload-port ${i}.local
    done
    ```

 * Monitor (ie. connect to serial)

    ```
    pio device monitor
    ```

 * List available devices

    ```
    pio device list
    ```
