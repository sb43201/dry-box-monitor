# ACE Dry Box Monitor

PlatformIO firmware for up to ten wireless AHT20 sensor nodes and one Hosyond/LCDWiki E32R35T central touchscreen.

For setup and day-to-day operation, see the [User Manual](USER_MANUAL.md).

## Hardware

### Sensor nodes (up to ten compatible units)

- Wemos D1 Mini ESP32 or Wemos D1 Mini ESP8266, 4 MB flash
- AHT20 breakout, or an AHT20/BMP280 combo module
- Optional 0.91-inch 128x32 SSD1306 I2C OLED (white)
- USB power

| AHT20 | Wemos D1 Mini ESP32 | Wemos D1 Mini ESP8266 |
|---|---|---|
| VCC | 3.3V | 3.3V |
| GND | GND | GND |
| SDA | GPIO21 | D2 / GPIO4 |
| SCL | GPIO22 | D1 / GPIO5 |

The node automatically detects an optional BMP280 at address `0x76` or `0x77`. AHT20-only and AHT20/BMP280 combo nodes use the same firmware. When present, BMP280 temperature and pressure are included in serial diagnostics and pressure is sent in the existing packet field.

The optional OLED shares the same I2C bus:

| SSD1306 OLED | Wemos D1 Mini ESP32 | Wemos D1 Mini ESP8266 |
|---|---|---|
| VCC | 3.3V | 3.3V |
| GND | GND | GND |
| SDA | GPIO21 | D2 / GPIO4 |
| SCL | GPIO22 | D1 / GPIO5 |

The firmware automatically detects a 128x32 display at I2C address `0x3C`. No setting or separate firmware is required. The OLED shows pairing state, ACE number, radio channel, temperature in both units, humidity, and transmission status.

Keep the AHT20 inside the dry box and, when practical, keep the Wemos board outside. Use a short four-wire cable. Do not power the module from 5 V.

### Central display

- Hosyond/LCDWiki E32R35T
- ESP32-WROOM-32E
- 3.5-inch 320x480 ST7796 TFT
- XPT2046 resistive touch
- Optional 8-32 GB FAT32 microSD card for logging and graph restoration

The display pin configuration matches the Plane Radar project.

The onboard microSD slot uses CS GPIO5, SCK GPIO18, MISO GPIO19, and MOSI GPIO23. It is on a separate SPI bus from the TFT and touch controller.

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

For a Wemos D1 Mini ESP32 sensor node:

```text
pio run -e sensor_wemos_d1_mini32 -t upload
```

For a Wemos D1 Mini ESP8266 sensor node:

```text
pio run -e sensor_wemos_d1_esp8266 -t upload
```

Repeat that upload for each connected sensor board, up to ten nodes.

## First startup and Wi-Fi

At every startup the touchscreen offers **USE WIFI** and **SKIP WIFI** for eight seconds. Tap **USE WIFI**, or make no selection, for normal network operation. Tap **SKIP WIFI** to run without Wi-Fi for the current boot only. Restarting the controller shows the choice again; offline mode is never saved permanently.

ESP-NOW sensor monitoring, the controller's local sensor, touchscreen readings, and history graphs continue in offline mode. The web dashboard, internet weather, and network time synchronization require Wi-Fi.

If Wi-Fi has not been configured and **USE WIFI** is selected, use a phone or computer to:

1. Join the access point shown on the display, named `DryBoxMonitor-Setup-XXXXXX`.
2. Open `http://192.168.4.1` if the captive portal does not open automatically.
3. Select a 2.4 GHz Wi-Fi network, enter its password, and optionally change the hostname.

The default local address is:

```text
http://drybox-monitor.local
```

The controller also prints its numeric IP address to the serial monitor. The `.local` address requires mDNS support on the viewing device; if it does not resolve, use the numeric IP shown by the router or serial monitor.

The local dashboard shows all ten nodes, live readings, packet age, paired/offline status, Wi-Fi channel, pairing controls, hostname configuration, and Wi-Fi reset. `/api/status` provides the same information as JSON and updates without reloading the page.

If the router temporarily disappears, the controller retries Wi-Fi every ten seconds and performs a stronger saved-credential reconnect after repeated failures. When the connection returns, it restores the dashboard, `.local` service, time synchronization, and weather updates automatically. If the router selects a different 2.4 GHz channel, the controller's ESP-NOW radio follows it and its beacons allow paired nodes to scan for and save the new channel without being paired again.

## Weather page

Tap **Weather** on the controller's bottom navigation bar. The page shows OpenWeather current conditions, feels-like temperature, outdoor humidity and wind, plus a four-day high/low and precipitation forecast. The controller's directly connected AHT20+BMP280 supplies the room temperature, humidity, and pressure.

Configure weather from the first-run portal or the **Weather setup** section at `http://drybox-monitor.local`:

- OpenWeather API key
- Location label
- Latitude and longitude
- POSIX timezone
- Fahrenheit/mph or Celsius/m/s

Weather refreshes every 20 minutes. The touchscreen **Refresh** button requests an immediate update. A free OpenWeather API key for the Current Weather and 5 Day / 3 Hour Forecast endpoints is required.

A node receives its ACE number during pairing rather than at compile time. Upload the environment matching each physical Wemos sensor board.

## Pairing and unpairing

Pair one node at a time:

1. On the display, open **Settings > Manage Sensor Nodes**, or choose an empty slot on the local web dashboard.
2. Tap the desired empty ACE slot.
3. Tap **Pair**. The controller listens for 60 seconds.
4. Power up an unpaired sensor node. It scans the 2.4 GHz channels and advertises automatically.
5. The slot changes to **Paired**. The assignment and controller MAC are saved in flash on both devices.

To remove a node, select its slot on the controller and tap **Unpair**. The controller clears the slot even if an offline node cannot receive the command. If the node is online, it also clears its saved assignment. This is the preferred recovery method.

### ESP32 hardware pairing reset

The Wemos D1 Mini ESP32 firmware supports a GPIO0 runtime reset:

1. Power and start the node normally. Do **not** ground GPIO0 during startup.
2. After boot, connect GPIO0 to GND for five seconds.
3. Remove the connection when serial output reports `[pair] Pairing cleared`.
4. Clear the old controller slot with **Unpair**, select the desired empty slot, and tap **Pair**.
5. Restart the node if it does not advertise immediately.

Grounding GPIO0 while powering on or resetting selects the ESP32 firmware-download boot mode; it does not perform the runtime pairing reset.

### Full flash erase and re-upload

Use this when an offline node cannot be reset normally. PlatformIO **Clean** only removes computer build files and does not clear pairing information on the board. Stop the serial monitor with `Ctrl+C`, connect only the node being reset, and replace `COM11` with its actual port.

Wemos D1 Mini ESP32:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e sensor_wemos_d1_mini32 -t erase --upload-port COM11
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e sensor_wemos_d1_mini32 -t upload --upload-port COM11
```

Wemos D1 Mini ESP8266:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e sensor_wemos_d1_esp8266 -t erase --upload-port COM11
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e sensor_wemos_d1_esp8266 -t upload --upload-port COM11
```

The ESP8266 does not use GPIO0 as a runtime unpair input, so controller **Unpair** or full erase/re-upload are its recovery methods. After erasing either board, clear the previous slot on the controller, start pairing on an empty slot, and restart the node.

## Operation

- Sensor nodes transmit immediately at startup and every 30 seconds.
- The controller uses Wi-Fi for the local dashboard and ESP-NOW for sensors. Pairing binds each physical node MAC to one controller slot and stores the router's current radio channel in the node.
- The controller broadcasts an ESP-NOW heartbeat approximately once per second. A paired node scans all channels after five seconds without that beacon and resumes when it finds its saved controller.
- A node is shown offline after two minutes without a valid packet.
- Tap a node for its detail screen.
- Use the `1-5` and `6-10` buttons to change touchscreen pages.
- Touch calibration runs automatically the first time the controller starts. Tap and release all four crosshairs. The calculated bounds are saved in flash. Use **Settings > Cal Touch** to repeat calibration later.
- Tap Settings to adjust the dry and humid thresholds. Values are stored in flash.
- Default thresholds are 30% RH for Dry and 45% RH for Humid. The range between is Check.
- Status does not depend on color alone: **DRY OK** uses a solid cyan marker, **CHECK !** uses a yellow striped marker, and **HUMID !!** uses a red cross-pattern marker. Offline sensors use a gray dashed outline.
- The local web dashboard uses the same colors, labels, and marker patterns as the touchscreen. Select **24h graph** on an online sensor card to view its five-minute temperature and humidity history without requiring internet access.

## microSD logging and graph restoration

Insert a FAT32-formatted microSD card before starting the controller. **Settings** shows the card's free space or **SD CARD: MISSING** when the controller is using RAM-only history. The web dashboard also reports free and total space. A missing or failed card does not stop ESP-NOW reception, the touchscreen, weather, or the in-memory graphs. The controller retries card detection once per minute.

Each new node packet sequence is buffered and written to `/logs/YYYY-MM-DD.csv`. Before network time is synchronized, rows are written to `/logs/unsynced.csv` with controller uptime. Duplicate radio retransmissions are not logged twice. The CSV contains node number, sequence, temperature in °C and °F, humidity, pressure, and packet flags.

The controller saves the five-minute graph buckets to `/history.bin` using a temporary file and rename operation. On the next startup it restores up to 24 hours of graph data for every node. History continues in RAM if the card is removed. For card safety, power down the controller before removing the microSD card.

When free space falls below 1 GB, the controller deletes the oldest completed `/logs/YYYY-MM-DD.csv` file once per second until at least 2 GB is free. It never deletes the current day's active log, `/logs/unsynced.csv`, or `/history.bin`. Settings displays the cleanup or low/full-space state, and each deletion is reported over serial.

The controller target uses PlatformIO's `huge_app.csv` partition to leave safe firmware space for SD support. This project uploads firmware over USB and does not use OTA firmware updates.

## Firmware identification

The controller firmware version and short Git commit are shown in three places:

- Startup serial output, for example `[firmware] version=1.0.0 commit=6a5e2eb`.
- The controller **Settings** screen.
- The web dashboard and `/api/status`.

PlatformIO generates the commit identifier automatically. Builds made with uncommitted tracked changes append `-dirty`, making it clear that the installed firmware does not exactly match a Git commit.

## Calibration

The display uses the Plane Radar project's default touch calibration. If touch positions are inaccurate, update `TOUCH_MIN_X`, `TOUCH_MAX_X`, `TOUCH_MIN_Y`, and `TOUCH_MAX_Y` near the top of `src/display/main.cpp`.

Per-node AHT20 correction values are `TEMPERATURE_OFFSET_C` and `HUMIDITY_OFFSET_RH` in `src/sensor/main.cpp`. For different corrections on each physical node, convert these to build flags in `platformio.ini` or change and upload one node at a time.
