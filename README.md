# ACE Dry Box Monitor

PlatformIO firmware for up to ten wireless AHT20 sensor nodes and one Hosyond/LCDWiki E32R35T central touchscreen.

For setup and day-to-day operation, see the [User Manual](USER_MANUAL.md).

## Hardware

### Sensor nodes (up to ten identical units)

- ESP32-C3 Super Mini, 4 MB flash
- AHT20 breakout, or an AHT20/BMP280 combo module
- Optional 0.91-inch 128x32 SSD1306 I2C OLED (white)
- USB power

| AHT20 | ESP32-C3 Super Mini |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO4 |
| SCL | GPIO5 |

The node automatically detects an optional BMP280 at address `0x76` or `0x77`. AHT20-only and AHT20/BMP280 combo nodes use the same firmware. When present, BMP280 temperature and pressure are included in serial diagnostics and pressure is sent in the existing packet field.

The optional OLED shares the same I2C bus:

| SSD1306 OLED | ESP32-C3 Super Mini |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO4 |
| SCL | GPIO5 |

The firmware automatically detects a 128x32 display at I2C address `0x3C`. No setting or separate firmware is required. The OLED shows pairing state, ACE number, radio channel, temperature in both units, humidity, and transmission status.

Keep the AHT20 inside the dry box and, when practical, keep the ESP32-C3 outside. Use a short four-wire cable. Do not power the module from 5 V.

### Central display

- Hosyond/LCDWiki E32R35T
- ESP32-WROOM-32E
- 3.5-inch 320x480 ST7796 TFT
- XPT2046 resistive touch

The display pin configuration matches the Plane Radar project.

### Controller local room sensor

Connect a separate AHT20 + BMP280 combo module inside the controller enclosure:

| Combo module | Hosyond controller |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO32 |
| SCL | GPIO25 |

GPIO32 and GPIO25 are separate from the TFT/touch SPI bus. This local sensor is not assigned an ACE node slot.

## Build and upload

Open this folder in VS Code with PlatformIO installed. Connect the display and upload:

```text
pio run -e display -t upload
```

For the sensor nodes, connect one ESP32-C3 at a time. Every node uses the same dynamically paired firmware:

```text
pio run -e sensor -t upload
```

Repeat that upload for each connected sensor board, up to ten nodes.

## First startup and Wi-Fi

At startup the touchscreen shows a welcome page. If Wi-Fi has not been configured, use a phone or computer to:

1. Join the access point shown on the display, named `DryBoxMonitor-Setup-XXXXXX`.
2. Open `http://192.168.4.1` if the captive portal does not open automatically.
3. Select a 2.4 GHz Wi-Fi network, enter its password, and optionally change the hostname.

The default local address is:

```text
http://drybox-monitor.local
```

The controller also prints its numeric IP address to the serial monitor. The `.local` address requires mDNS support on the viewing device; if it does not resolve, use the numeric IP shown by the router or serial monitor.

The local dashboard shows all ten nodes, live readings, packet age, paired/offline status, Wi-Fi channel, pairing controls, hostname configuration, and Wi-Fi reset. `/api/status` provides the same information as JSON and updates without reloading the page.

## Weather page

Tap **Weather** on the controller's bottom navigation bar. The page shows OpenWeather current conditions, feels-like temperature, outdoor humidity and wind, plus a four-day high/low and precipitation forecast. The controller's directly connected AHT20+BMP280 supplies the room temperature, humidity, and pressure. ACE 1 remains a normal dry-box node.

Configure weather from the first-run portal or the **Weather setup** section at `http://drybox-monitor.local`:

- OpenWeather API key
- Location label
- Latitude and longitude
- POSIX timezone
- Fahrenheit/mph or Celsius/m/s

Weather refreshes every 20 minutes. The touchscreen **Refresh** button requests an immediate update. A free OpenWeather API key for the Current Weather and 5 Day / 3 Hour Forecast endpoints is required.

A node receives its ACE number during pairing rather than at compile time. Upload the same `sensor` environment to every physical sensor node.

## Pairing and unpairing

Pair one node at a time:

1. On the display, open **Settings > Manage Sensor Nodes**, or choose an empty slot on the local web dashboard.
2. Tap the desired empty ACE slot.
3. Tap **Pair**. The controller listens for 60 seconds.
4. Power up an unpaired sensor node. It scans the 2.4 GHz channels and advertises automatically.
5. The slot changes to **Paired**. The assignment and controller MAC are saved in flash on both devices.

To remove a node from the controller, select its slot and tap **Unpair**. The controller sends an unpair command to the node and clears the slot. If the node is offline and cannot receive that command, power it normally and hold its **BOOT** button for five seconds. Holding BOOT for five seconds at runtime always erases the node's saved controller and returns it to pairing mode.

Do not hold BOOT while applying power; on an ESP32-C3 that selects the firmware-download boot mode. Press and hold it only after the node has started.

## Operation

- Sensor nodes transmit immediately at startup and every 30 seconds.
- The controller uses Wi-Fi for the local dashboard and ESP-NOW for sensors. Pairing binds each physical node MAC to one controller slot and stores the router's current radio channel in the node.
- A node is shown offline after two minutes without a valid packet.
- Tap a node for its detail screen.
- Use the `1-5` and `6-10` buttons to change touchscreen pages.
- Touch calibration runs automatically the first time the controller starts. Tap and release all four crosshairs. The calculated bounds are saved in flash. Use **Settings > Cal Touch** to repeat calibration later.
- Tap Settings to adjust the dry and humid thresholds. Values are stored in flash.
- Default thresholds are 30% RH for Dry and 45% RH for Humid. The range between is Check.

## Calibration

The display uses the Plane Radar project's default touch calibration. If touch positions are inaccurate, update `TOUCH_MIN_X`, `TOUCH_MAX_X`, `TOUCH_MIN_Y`, and `TOUCH_MAX_Y` near the top of `src/display/main.cpp`.

Per-node AHT20 correction values are `TEMPERATURE_OFFSET_C` and `HUMIDITY_OFFSET_RH` in `src/sensor/main.cpp`. For different corrections on each physical node, convert these to build flags in `platformio.ini` or change and upload one node at a time.
