# ACE Dry Box Monitor User Manual

## 1. System overview

The ACE Dry Box Monitor consists of:

- One central ESP32 touchscreen controller.
- One local AHT20/BMP280 combo sensor connected to the controller.
- Up to ten wireless ACE dry-box sensor nodes.
- An optional OpenWeather internet weather display.
- A local web dashboard available from a phone or computer on the same Wi-Fi network.

The wireless nodes report dry-box temperature and humidity to the controller through ESP-NOW. The controller uses Wi-Fi for time synchronization, weather, and the web dashboard.

### Optional sensor-node OLED

Each sensor node can use a 0.91-inch 128x32 SSD1306 I2C OLED with a white display. Connect it in parallel with the AHT20:

| OLED | ESP32-C3 Super Mini |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO4 |
| SCL | GPIO5 |

The expected I2C address is `0x3C`. The node detects the OLED automatically. It shows pairing status, assigned ACE number, Wi-Fi channel, temperature in °C and °F, humidity, packet number, and transmission status. The node operates normally if no OLED is connected.

Each node may use either a standalone AHT20 or an AHT20/BMP280 combo module on the same GPIO4/GPIO5 I2C bus. The BMP280 is detected automatically at address `0x76` or `0x77`. A combo node adds raw BMP280 temperature and pressure to serial diagnostics and transmits pressure in the existing packet field; a standalone AHT20 node reports the BMP280 as not installed and otherwise works normally.

## 2. Controls and indicators

The main touchscreen shows five ACE nodes at a time. Each online node displays:

- Temperature in both °C and °F.
- Relative humidity.
- A color-coded condition: **DRY**, **CHECK**, or **HUMID**.
- Online, offline, or sensor-error status.

The top of the main screen shows the number of online nodes and the controller's Wi-Fi IP address. Use **1-5** and **6-10** to switch node pages, **WEATHER** for weather and room conditions, and **SET** for controller settings.

Tap a node card to open its detail page. The detail page shows temperature in both units, humidity, pressure when available, packet number, and reading age.

## 3. First startup

On first startup, follow the four touchscreen calibration targets. Tap and release each crosshair accurately. Calibration is saved automatically.

If Wi-Fi is not configured, the controller creates an access point named:

`DryBoxMonitor-Setup-XXXXXX`

Using a phone or computer:

1. Join the setup network shown on the touchscreen.
2. Wait for the setup portal to open, or browse to `http://192.168.4.1`.
3. Select a 2.4 GHz Wi-Fi network and enter its password.
4. Enter the optional hostname and weather settings.
5. Save and allow the controller to connect.

The default dashboard address is `http://drybox-monitor.local`. If that address does not open, use the numeric IP shown at the top of the main touchscreen.

## 4. Main screen

Node colors use the configured humidity limits:

- **DRY:** humidity is at or below the Dry limit.
- **CHECK:** humidity is between the Dry and Humid limits.
- **HUMID:** humidity is at or above the Humid limit.
- **OFFLINE:** no valid packet has arrived for two minutes.
- **SENSOR ERROR:** the node is communicating but its sensor reading is invalid.

Open **SET** to adjust the Dry and Humid thresholds. Tap **SAVE & BACK** to store them.

## 5. Weather and room sensor screen

Tap **WEATHER** to view:

- Current outdoor temperature.
- Selected outdoor unit, shown as °C or °F.
- Feels-like temperature, humidity, wind, and conditions.
- Four-day high/low and precipitation forecast.
- Local date and time for the configured timezone.
- Controller-room AHT20 temperature and humidity.
- Controller-room BMP280 pressure.

The room sensor temperature is always shown in both °C and °F.

Touchscreen buttons:

- **BACK:** return to the node overview.
- **SHOW C / SHOW F:** change the outdoor weather and forecast units. The choice is saved.
- **REFRESH:** request a new OpenWeather update.

Weather normally refreshes every 20 minutes. Internet access and a valid OpenWeather API key are required.

## 6. Web dashboard

From a device on the same network, open:

`http://drybox-monitor.local`

If mDNS is unavailable, use the numeric IP shown on the touchscreen.

The dashboard provides:

- All ten node readings and statuses.
- Temperature in °C and °F.
- Humidity, packet age, and pairing status.
- Pair and unpair controls.
- Weather and local sensor configuration.
- Hostname configuration.
- Wi-Fi reset.

### Weather setup

Enter:

- OpenWeather API key.
- Location name.
- Latitude and longitude.
- POSIX timezone.
- Outdoor weather unit preference.
- AHT20 display temperature offset.

The AHT20 offset is entered in °C and accepts values from -20.0 to +20.0. It corrects only the controller's local AHT20 temperature. It does not change humidity, BMP280 pressure, outdoor weather, or wireless ACE node readings.

Example: if a reference thermometer reads 22.0 °C and the local AHT20 reads 22.7 °C, enter `-0.7`.

## 7. Pairing wireless sensor nodes

All wireless nodes use the same sensor firmware. Pair one node at a time:

1. Open **SET** on the touchscreen.
2. Tap **PAIR NODES**.
3. Select an empty ACE slot.
4. Tap **PAIR**. The controller listens for 60 seconds.
5. Power up the unpaired sensor node.
6. Wait for the slot to show **PAIRED**.

The controller and node save the assignment in flash.

You can also start pairing from an empty slot on the web dashboard.

## 8. Unpairing and resetting a node

To unpair from the touchscreen:

1. Open **SET > PAIR NODES**.
2. Select the paired slot.
3. Tap **UNPAIR**.

If the node is online, the controller tells it to erase its pairing. If it was offline, power it normally and hold its **BOOT** button for five seconds to clear its saved controller.

Do not hold BOOT while applying power; that enters firmware-download mode. Hold it only after the node has started.

## 9. Touchscreen recalibration

Open **SET** and tap **CAL TOUCH**. Tap and release all four targets. The new calibration is saved automatically.

Recalibrate if buttons respond in the wrong place or are difficult to select.

## 10. Serial monitor diagnostics

Connect the controller by USB and open a serial monitor at 115200 baud.

Every local-sensor cycle reports:

- AHT20 raw temperature in °C and °F.
- AHT20 corrected temperature in °C and °F.
- Applied AHT20 offset.
- AHT20 relative humidity.
- BMP280 raw temperature in °C and °F.
- BMP280 pressure.
- Wi-Fi SSID, RSSI in dBm, channel, IP address, and hostname.

Each received wireless packet reports its node number, MAC address, sequence number, temperature in °C and °F, humidity, pressure, and flags.

A less-negative RSSI is stronger. For example, -45 dBm is stronger than -75 dBm.

## 11. Wi-Fi changes

To change networks:

1. Open the web dashboard.
2. Find **Controller**.
3. Select **Change Wi-Fi** and confirm.
4. After restart, join the new `DryBoxMonitor-Setup-XXXXXX` access point.
5. Complete setup again.

The controller and wireless sensors must share the router's current 2.4 GHz channel for ESP-NOW operation. The paired nodes store the channel provided by the controller.

## 12. Troubleshooting

### Dashboard does not open

- Confirm the controller says **WiFi** followed by an IP address.
- Browse directly to that numeric IP.
- Confirm the phone or computer is on the same network.
- Disable cellular data temporarily if the phone keeps leaving the local network.

### Weather says setup needed or waiting

- Verify the OpenWeather API key, latitude, and longitude.
- Confirm the controller has Wi-Fi internet access.
- Newly created OpenWeather keys may take time to activate.
- Tap **REFRESH** after correcting settings.

### Date or time is wrong

- Verify the POSIX timezone in Weather setup.
- Confirm internet access so NTP can synchronize.
- Restart the controller after changing network configuration.

### Node is offline

- Confirm the node has power.
- Wait up to 30 seconds for its next transmission.
- Confirm the router's 2.4 GHz channel has not changed.
- Unpair and pair the node again if necessary.

### Local sensor error

- Confirm the combo module uses 3.3 V.
- Confirm SDA is GPIO32 and SCL is GPIO25.
- Check ground and connector continuity.
- The controller supports BMP280 addresses 0x76 and 0x77.

## 13. Normal update intervals

- Wireless ACE node transmission: every 30 seconds.
- Local AHT20/BMP280 reading: every 30 seconds.
- Offline indication: after two minutes without a valid node packet.
- OpenWeather update: every 20 minutes.
