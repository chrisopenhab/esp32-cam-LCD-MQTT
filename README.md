# ZaehlerCam – ESP32-CAM meter reader for blinking 7-segment LCDs

🇩🇪 [Deutsche Version](README.de.md) · License: GPL-3.0-or-later

An ESP32-CAM looks at the small LCD of a meter (e.g. a heat/kWh meter) whose digits are only visible for about **one second every few seconds**. The firmware detects when the display switches on, reads the 7-segment digits, takes a majority vote over several frames and publishes the reading via **MQTT**. Everything – camera, display frame, digit grid, filters, MQTT, LED ring, firmware update, diagnostics – is configured in a **web interface** (German/English) served by the ESP32 itself. No PC software is needed.

> Status: developed iteratively on real hardware (AI-Thinker ESP32-CAM, 7-digit kWh meter, WS2812B ring). The sketch could not be compiled by its author's tooling in every iteration – if the Arduino IDE reports an error, please open an issue and paste the first lines of the message.

## Features

- **Web UI** (4 pages, German / English selectable in the header): system, camera, display & filter, diagnostics
- **OTA firmware update** through the web UI (raw `.bin` upload)
- **Camera settings page** with live preview, all sensor parameters, resolution QVGA…SXGA
- **Display frame drawn with the mouse** (move, scale at the corners, rotation in 0.05° steps) and a **segment grid per digit** (7 segments a–g, slant/italic, gaps, margins, per-segment manual correction)
- **Digit patterns** (which segments form 0…9) editable; error tolerance for missing segments
- **Filters** for clean black/white segment detection: global threshold (Otsu/manual), local contrast, *background compensation* (adaptive, recommended)
- **Auto-calibration** of rotation, frame and digit grid from live images
- **Display-on detection** with six methods, including *“digits recognised”* (robust against exposure changes). A **Hold** button freezes the stream so you can label images as *display ON / OFF*; the detection is then **calibrated automatically** from those samples
- **Lock** checkbox: protects frame, rotation and grid against accidental mouse clicks
- **MQTT** (switchable): value, raw text, JSON, display state, online/offline (LWT); retain and “only on change” options
- **Plausibility check**: a counter may only count up; smaller values and too large jumps are discarded, missing/unreadable digits can be filled in from the last value, incomplete readings can optionally be published (never as `value`)
- **openHAB examples**: things, items, persistence (rrd4j, mapdb, InfluxDB) and rules
- **LED ring** control (WS2812B/NeoPixel or simple PWM LEDs), configurable pin, brightness, colour
- **Settings export/import** (JSON)
- **Diagnostics page**: image package (settings + measurements + live image + last “on” image, with/without overlay) and **recording** (video `.webm` and/or ZIP of single frames with a `messwerte.csv` of per-frame measurements) for a configurable duration – send it to someone to tune the detection
- Detailed **serial debug output** (`printf`, in German)

## Hardware

| Part | Notes |
|---|---|
| ESP32-CAM, **AI-Thinker**, with PSRAM | other boards need different camera pins |
| LED ring WS2812B (e.g. 8 LEDs, “WCMCU-2812B-8”) | data → one of GPIO 2, 4, 12, 13, 14, 15, 33 (default 12; 12/2/15 are strapping pins) |
| USB-serial adapter (FTDI/CH340) | only for the first flash |
| 5 V power supply, ≥ 1 A | ring + camera + Wi-Fi need current; weak supplies cause resets |

Mount the camera ~10–15 cm in front of the LCD, **focus the lens by hand** (turn the lens barrel until the digits are sharp – this matters more than any software setting). Avoid glare on the LCD: tilt the ring/camera slightly or reduce the LED brightness.

## Software requirements

- Arduino IDE 2.x (or arduino-cli) with **arduino-esp32** core 2.0.x or 3.x (tested with 3.3.12)
- Board: **AI Thinker ESP32-CAM**, PSRAM: **Enabled**
- Partition scheme: **Minimal SPIFFS (1.9 MB APP with OTA)** (or Default 4MB with spiffs)
- Libraries (Library Manager): **ArduinoJson 7.x**, **PubSubClient**, **Adafruit NeoPixel**

## Building the firmware (`.bin`)

**1. Install the Arduino IDE 2.x** from <https://www.arduino.cc/en/software>.

**2. Add the ESP32 board package**
- *File → Preferences → Additional boards manager URLs*, add:
  `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
- *Tools → Board → Boards Manager…* → search **“esp32”** → install **“esp32 by Espressif Systems”** (3.x; developed and tested with 3.3.12, 2.0.17 also works). The download is large (several hundred MB), be patient.

**3. Install the libraries** (*Sketch → Include Library → Manage Libraries…*)

| Library | Author | Version |
|---|---|---|
| **ArduinoJson** | Benoit Blanchon | 7.x (not 6.x) |
| **PubSubClient** | Nick O'Leary | current |
| **Adafruit NeoPixel** | Adafruit | current |

Everything else (`WiFi`, `LittleFS`, `ESPmDNS`, `Update`, camera driver) is part of the ESP32 package.

**4. Select the board and options** (*Tools* menu)

| Setting | Value |
|---|---|
| Board | **ESP32 Arduino → AI Thinker ESP32-CAM** |
| PSRAM | **Enabled** (if the menu shows it) |
| Partition Scheme | **Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)** |
| Port | the USB-serial adapter (only needed for upload) |

**5. Compile and export the binary**
- Open `ZaehlerCam/ZaehlerCam.ino` (the folder name must equal the file name).
- *Sketch → Export Compiled Binary* (Ctrl+Alt+S). This compiles without uploading.
- The files appear in the sketch folder under `build/esp32.esp32.esp32cam/`:

| File | Use |
|---|---|
| **`ZaehlerCam.ino.bin`** | **the application – upload this one via the web UI (OTA)** |
| `ZaehlerCam.ino.bootloader.bin`, `ZaehlerCam.ino.partitions.bin` | only for manual flashing together with the app |
| `ZaehlerCam.ino.merged.bin` | complete image for a blank chip (flash at address `0x0`, e.g. with esptool or the Espressif web flasher) |

**6. Flash it**
- **First time / blank chip:** *Sketch → Upload* with GPIO 0 tied to GND during reset (see *Installation*), or flash the `merged.bin` at `0x0`.
- **Afterwards:** open the web UI → *1 · System → Firmware update* → choose `ZaehlerCam.ino.bin` → *Upload & flash*. The device restarts; check version/build in the header.

**Command-line alternative (arduino-cli)** – not run by the author of this documentation, adapt if your version differs:

```bash
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install "ArduinoJson" "PubSubClient" "Adafruit NeoPixel"
arduino-cli compile --fqbn esp32:esp32:esp32cam --board-options PartitionScheme=min_spiffs --export-binaries ZaehlerCam
# result: ZaehlerCam/build/esp32.esp32.esp32cam/ZaehlerCam.ino.bin
```

**Typical build problems**

| Message | Fix |
|---|---|
| `ArduinoJson.h: No such file` | install the library (step 3) |
| `String(double, unsigned char)` is ambiguous | old sketch version – use the current one |
| `Sketch too big` / partition error | wrong partition scheme (step 4) |
| Brown-out / reset loops when running | weak 5 V supply, see *Hardware* |

## Installation

1. Copy the folder `ZaehlerCam/` (the sketch folder name must equal the `.ino` name) into your Arduino sketchbook and open `ZaehlerCam.ino`.
2. **First flash via USB**: connect GPIO 0 to GND, reset, upload; remove the jumper and reset again.
3. **Wi-Fi setup**: without stored credentials the device opens the access point **`ZaehlerCam-Setup`** (open) → <http://192.168.4.1/> → page *1 · System* → enter SSID/password → *Save* → *Restart*.
4. Open the web UI at the IP shown in the serial log or at <http://zaehlercam.local/> (hostname configurable).
5. **Later updates over the air**: Arduino IDE → *Sketch → Export compiled binary*. In the export folder there are several `.bin` files – upload **`ZaehlerCam.ino.bin`** (*not* the `…bootloader…`, `…partitions…` or `…merged…` files) on page 1 under *Firmware update*. The page shows the running version/build so you can verify the update.

## Setting it up – recommended workflow

1. **Page 1 · System**: Wi-Fi, MQTT, LED (type *WS2812B*, pin, number of LEDs, brightness). Save, restart.
2. **Page 2 · Camera**: enable the camera, pick a resolution (VGA/SVGA), adjust brightness/contrast/exposure so the digits are visible without blown-out highlights. Sharpness −2…0 and moderate contrast usually work best.
3. **Page 3 · Display & Filter**:
   1. Drag the blue frame around the digits (slightly larger is fine), set the number of digits.
   2. Press **🎯 Auto-calibration** (keep the display on for ~30 s if possible) or fine-tune *rotation*, gap/width/height/slant and move digits/segments manually. The green/red segment overlay shows what is detected; the *filter preview* shows what the evaluation sees.
   3. Tick **🔒 Lock frame & grid** once it fits.
   4. Check the *digit patterns* (a–g) and set the *error tolerance* (1 tolerates one wrong/missing segment per digit – useful when a segment is faint).
   5. **Display-on detection**: choose *Digits recognised* (minimum number of valid digits, e.g. 5 of 7). For automatic calibration press **⏸ Hold** while the display is **off**, click *Display is OFF*; press *Resume*, hold again while the digits are visible, click *Display is ON*. Record at least one image per state (better 3+, with different light), then **🎯 Calibrate “display on” detection**. The result and its separation margin are shown; unwanted samples can be deleted individually or all reset.
   6. **💾 Save** (the header message tells whether values are only applied live or stored).
4. **MQTT** is optional; check the topics below with an MQTT client.
5. **Page 4 · Diagnostics** if something does not work (see below).

## MQTT topics

`<base>` is the *base topic* (default `zaehlercam`).

| Topic | Content |
|---|---|
| `<base>/value` | reading as number with the configured decimal places (retain optional) |
| `<base>/raw` | digits as recognised, without decimal point |
| `<base>/json` | `{"value":…, "raw":"…", "valid":true, "reconstructed":false, "uptime":…}`; for incomplete readings (option) `{"value":null, "raw":"00?2123", "valid":false, …}` |
| `<base>/display` | `ON` / `OFF` – display detected on/off |
| `<base>/rejected` | a complete reading that the plausibility check discarded (digits as text) |
| `<base>/seen` | only when the display was on but **no valid reading** was obtained: the last text seen, e.g. `00?2123` |
| `<base>/status` | `online` / `offline` (last will, retained) |
| `<base>/diag` | health JSON every 5 min and after connecting: firmware, uptime, free heap / minimum heap, PSRAM, RSSI, Wi-Fi reconnects, last reset reason |

Options: *Always send* (every reading) or only when the value changed; *Retain* for value/raw.

## Plausibility check

A reading is only published as `value` if **all** digits were read as valid patterns. Values containing `?` are therefore never published as `value` – parsing `00?2123` as a number would silently give a wrong figure. On top of that (page 1, *Plausibility check*, enabled by default):

- **Counter may only count up** – a smaller value than the last accepted one is discarded and published on `<base>/rejected` instead.
- **Max. increase per reading** (default 100 counts, `0` = unlimited) – larger jumps (typically one misread high digit) are discarded.
- **Confirmations for a new baseline** (default 3) – if the *same* discarded value is read that many times in a row it becomes the new baseline (meter replaced, or the first value after power-up was wrong).
- **Fill missing digits from the last value** (default on, needs a max. increase > 0) – for a partial reading such as `00?2195` the unreadable digit is taken from the last value (`0022190` → `0022195`). The result is only used if it passes the checks above and is flagged `"reconstructed":true` in the JSON.
- **Send incomplete values (with ?)** (default off) – if nothing could be completed, the partial text is published on `<base>/raw` and `<base>/json` with `"valid":false`, **never on `<base>/value`**. Useful for debugging, not for automation.

The last accepted value is kept in RAM only: after a restart the first complete reading is accepted without comparison. Counts are compared as integers of the digits, so decimal places do not matter.

## How a reading is taken

`OFF → display on detected → (settling time, skipped for the “digits recognised” method) → N measurements (majority vote) → publish → wait until display off`. A measurement counts only if **all** digits are valid patterns. Optionally the display can be re-read every *x* seconds while it stays on. If the display goes off before enough samples were taken, the best available vote is published.

## Display-on detection methods

| Method | Triggers when | Use when |
|---|---|---|
| Brightness ≥ / ≤ value | mean brightness of the frame above/below a value | stable lighting |
| Contrast ≥ value | standard deviation in the frame | digits give strong contrast |
| Brightness / contrast jump (adaptive) | jump versus a slowly tracked baseline | lighting drifts |
| **Digits recognised** (recommended) | at least *N* digits give a valid pattern | works independently of exposure; needs a good grid |

## Filters

*Background compensation* (default) compares every pixel with the local mean (radius in px) and marks a segment as lit when ≥ *x* % of its pixels are darker than the surroundings by more than a threshold (automatic or manual). *Local contrast* compares segment strips with their neighbours; *global threshold* uses Otsu or a fixed value. *Invert* = dark segments on a light background (typical LCD).

## openHAB

Ready-to-use examples are in [`examples/openhab/`](examples/openhab/): MQTT things/channels (incl. availability from the last will and JSON field channels), items, persistence for rrd4j / mapdb / InfluxDB (`.persist`) and rules (24 h / 7 d consumption, watchdog, log of rejected values). They were written from the openHAB documentation and not tested against a running installation.

## Diagnostics

- **Image package**: downloads several files at once – settings (no passwords), measurements as JSON, the live image and the last “display on” image, each with and without overlay.
- **Recording**: records for *n* seconds as fast as possible – video (`.webm` with overlay) and/or a ZIP of single JPEGs with `messwerte.csv` (per frame: display on/off, brightness, contrast, threshold, digits valid, recognised text, segment masks). The browser pulls frames over Wi-Fi, so ~1–8 frames/s are typical; the *firmware itself* evaluates every ~120 ms.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Values appear on `<base>/rejected` | the plausibility check discarded them (smaller than last value / jump too large); a wrong baseline is replaced after the configured number of identical readings, or raise *max. increase* |
| No value on MQTT, status/display arrive | no frame had **all** digits valid → check `<base>/seen` and the serial log line `Display war an, aber KEIN gueltiger Wert…`; fix grid, filter, increase *error tolerance* to 1 |
| Digits `?` | segment grid shifted; faint segment (often the thin top bar `a`); try tolerance 1, lower manual threshold (e.g. 4), re-calibrate on a sharp frame |
| Auto-calibration fails | image blurry or glare → focus lens, reduce LED brightness, draw frame closer around the digits, calibrate while the display stays on |
| Display never “on” | use method *Digits recognised*; make sure the grid reads at least *N* digits in the *held* ON image |
| Device unreachable after hours | check the reset reason in the header / `<base>/diag` (`brownout` → stronger 5 V supply, short thick cables, 470–1000 µF capacitor near the board; `Watchdog` → see serial log; falling `minheap` → memory leak, report it with the serial log). The firmware reconnects Wi-Fi and reboots itself, so a permanent outage points to power or the router |
| UI very slow / stalls | weak power supply or poor Wi-Fi; close other tabs with the live preview |
| LED ring does not switch off | wrong LED type (ring is WS2812B, not PWM) |
| OTA shows old version | uploaded the wrong `.bin`; hard-reload the page and compare version/build |

## Stability / self-monitoring

- **Wi-Fi supervision**: if the connection is lost the firmware reconnects every 30 s; after 5 minutes without Wi-Fi it restarts itself.
- **Watchdog task**: if the main loop does not respond for 60 s (e.g. a stuck camera or lock) or free heap stays below 12 kB for 30 s, the device restarts itself.
- The header of the web UI and `<base>/diag` show uptime, heap (current/minimum), Wi-Fi reconnects and the **reset reason** (power-on, software restart + watchdog cause, crash/panic, **brownout** = supply voltage dip). The serial log prints it at boot (`Neustart-Grund`). A steadily falling minimum heap indicates a memory leak; `brownout` points to the power supply.

## Security

The web interface has **no authentication**. Run it only in a trusted network. The *Export* button writes Wi-Fi/MQTT passwords into the JSON – do not publish exported files (see `.gitignore`).

## Known limitations

7-segment LCDs only (up to 8 digits); perspective distortion (trapezoid) is not corrected, only rotation; very blurry or glare-dominated images cannot be read; the serial log is German; text burned into the diagnostic overlay images is German.

## Repository layout

```
ZaehlerCam/ZaehlerCam.ino     firmware + embedded web UI (single file)
examples/                     example settings of the author's setup (frame/grid/filter/detection only)
examples/openhab/             openHAB things, items, persistence, rules
README.md, README.de.md       documentation
LICENSE                       GNU GPL v3
```

The example settings are tuned to one camera position and one meter; use them only as a reference of the JSON structure. Partial JSON files are fine for import – missing keys keep their current value.

## License

This project is licensed under the **GNU General Public License v3.0 or later** – see [`LICENSE`](LICENSE). You may use, modify and redistribute it; derived works must be distributed under the same license with their source code. There is no warranty.

The firmware uses these libraries, which keep their own licenses: ArduinoJson, PubSubClient, Adafruit NeoPixel and the arduino-esp32 core (please check their current license terms when redistributing binaries).
