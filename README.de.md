# ZaehlerCam – ESP32-CAM liest blinkende 7-Segment-LCDs

🇬🇧 [English version](README.md)

Eine ESP32-CAM schaut auf das kleine LCD eines Zählers (z. B. Wärme-/kWh-Zähler), dessen Ziffern nur etwa **eine Sekunde alle paar Sekunden** sichtbar sind. Die Firmware erkennt, wann die Anzeige angeht, liest die 7-Segment-Ziffern, bildet über mehrere Bilder eine Mehrheitsentscheidung und veröffentlicht den Zählerstand per **MQTT**. Alles – Kamera, Display-Rahmen, Ziffern-Raster, Filter, MQTT, LED-Ring, Firmware-Update, Diagnose – wird in einer **Weboberfläche** (Deutsch/Englisch) eingestellt, die der ESP32 selbst bereitstellt. Es ist keine PC-Software nötig.

> Stand: iterativ an echter Hardware entwickelt (AI-Thinker ESP32-CAM, 7-stelliger kWh-Zähler, WS2812B-Ring). Falls die Arduino IDE einen Fehler meldet, bitte ein Issue öffnen und die ersten Zeilen der Meldung einfügen.

## Funktionen

- **Weboberfläche** mit 4 Seiten (Sprache Deutsch/English oben rechts umschaltbar): System, Kamera, Display & Filter, Diagnose
- **OTA-Firmware-Update** über die Weboberfläche (`.bin`-Upload)
- **Kameraeinstellungen** mit Livebild, allen Sensorparametern, Auflösung QVGA…SXGA
- **Display-Rahmen mit der Maus** aufziehen (verschieben, an den Ecken skalieren, Drehung in 0,05°-Schritten) und **Segment-Raster pro Ziffer** (7 Segmente a–g, Kursiv, Abstände, Ränder, manuelle Korrektur je Segment)
- **Ziffernmuster** (welche Segmente 0…9 bilden) editierbar; Fehlertoleranz für fehlende Segmente
- **Filter** für sauberes Schwarz/Weiß: globale Schwelle (Otsu/manuell), lokaler Kontrast, *Hintergrund-Ausgleich* (adaptiv, empfohlen)
- **Auto-Kalibrierung** von Drehung, Rahmen und Ziffernraster aus Livebildern
- **Display-an-Erkennung** mit sechs Verfahren, u. a. *„Ziffern erkannt“* (unabhängig von der Belichtung). Mit **Hold** wird der Stream angehalten, und die Bilder lassen sich als *Anzeige AN / AUS* markieren; daraus wird die Erkennung **automatisch kalibriert**
- **Sperre**-Checkbox: schützt Rahmen, Drehung und Raster vor versehentlichen Mausklicks
- **MQTT** (abschaltbar): Wert, Rohtext, JSON, Display-Zustand, online/offline (LWT); Retain und „nur bei Änderung“
- **Plausibilitätsprüfung**: Ein Zähler darf nur hochzählen; kleinere Werte und zu große Sprünge werden verworfen, fehlende/unleserliche Ziffern lassen sich aus dem letzten Wert ergänzen, unvollständige Lesungen können optional gesendet werden (nie als `value`)
- **openHAB-Beispiele**: Things, Items, Persistence (rrd4j, mapdb, InfluxDB) und Regeln
- **LED-Ring** (WS2812B/NeoPixel oder einfache PWM-LEDs): Pin, Helligkeit, Farbe einstellbar
- **Export/Import** der Einstellungen (JSON)
- **Diagnose-Seite**: Bilder-Paket (Einstellungen + Messwerte + Livebild + letztes „an“-Bild, mit/ohne Einzeichnung) und **Aufnahme** (Video `.webm` und/oder ZIP aus Einzelbildern mit `messwerte.csv`) mit einstellbarer Dauer – zum Weitergeben, um die Erkennung zu optimieren
- Ausführliche **serielle Debug-Ausgabe** (`printf`, auf Deutsch)

## Hardware

| Teil | Hinweis |
|---|---|
| ESP32-CAM **AI-Thinker** mit PSRAM | andere Boards brauchen andere Kamera-Pins |
| LED-Ring WS2812B (z. B. 8 LEDs, „WCMCU-2812B-8“) | Daten → GPIO 2, 4, 12, 13, 14, 15 oder 33 (Standard 12; 12/2/15 sind Strapping-Pins) |
| USB-Seriell-Adapter (FTDI/CH340) | nur fürs erste Flashen |
| 5-V-Netzteil, ≥ 1 A | Ring + Kamera + WLAN brauchen Strom; schwache Netzteile führen zu Resets |

Kamera ca. 10–15 cm vor dem LCD montieren und das **Objektiv von Hand scharfstellen** (Objektivtubus drehen, bis die Ziffern scharf sind – das bringt mehr als jede Software-Einstellung). Spiegelungen auf dem LCD vermeiden: Ring/Kamera leicht neigen oder die LED-Helligkeit verringern.

## Software-Voraussetzungen

- Arduino IDE 2.x (oder arduino-cli) mit **arduino-esp32** Core 2.0.x oder 3.x (getestet mit 3.3.12)
- Board: **AI Thinker ESP32-CAM**, PSRAM: **Enabled**
- Partitionsschema: **Minimal SPIFFS (1.9 MB APP with OTA)** (oder Default 4MB with spiffs)
- Bibliotheken (Library-Manager): **ArduinoJson 7.x**, **PubSubClient**, **Adafruit NeoPixel**

## Installation

1. Ordner `ZaehlerCam/` (Ordnername = Name der `.ino`) in das Arduino-Sketchbook kopieren und `ZaehlerCam.ino` öffnen.
2. **Erstes Flashen per USB**: GPIO 0 mit GND verbinden, Reset, hochladen; Brücke entfernen und erneut Reset.
3. **WLAN einrichten**: Ohne gespeicherte Zugangsdaten öffnet das Gerät den Access Point **`ZaehlerCam-Setup`** (offen) → <http://192.168.4.1/> → Seite *1 · System* → SSID/Passwort eintragen → *Speichern* → *Neustart*.
4. Weboberfläche über die IP aus dem seriellen Log oder <http://zaehlercam.local/> öffnen (Hostname einstellbar).
5. **Spätere Updates per WLAN**: Arduino IDE → *Sketch → Kompilierte Binärdatei exportieren*. Im Exportordner liegen mehrere `.bin`-Dateien – hochladen auf Seite 1 unter *Firmware-Update* muss **`ZaehlerCam.ino.bin`** (*nicht* `…bootloader…`, `…partitions…` oder `…merged…`). Die Seite zeigt Version/Build der laufenden Firmware zur Kontrolle.

## Einrichtung – empfohlener Ablauf

1. **Seite 1 · System**: WLAN, MQTT, LED (Typ *WS2812B*, Pin, Anzahl, Helligkeit). Speichern, Neustart.
2. **Seite 2 · Kamera**: Kamera aktivieren, Auflösung wählen (VGA/SVGA), Helligkeit/Kontrast/Belichtung so einstellen, dass die Ziffern ohne überstrahlte Stellen sichtbar sind. Schärfe −2…0 und mäßiger Kontrast funktionieren meist am besten.
3. **Seite 3 · Display & Filter**:
   1. Blauen Rahmen um die Ziffern ziehen (etwas größer ist in Ordnung), Anzahl der Ziffern setzen.
   2. **🎯 Auto-Kalibrierung** drücken (möglichst ~30 s die Anzeige anlassen) oder *Drehung*, Abstand/Breite/Höhe/Kursiv sowie Ziffern/Segmente von Hand anpassen. Die grün/rote Segment-Einblendung zeigt die Erkennung, die *Filter-Vorschau* zeigt, was die Auswertung sieht.
   3. Wenn alles passt: **🔒 Rahmen & Raster sperren**.
   4. *Ziffern-Muster* (a–g) prüfen und *Fehlertoleranz* setzen (1 toleriert ein falsches/fehlendes Segment pro Ziffer – nützlich bei schwachen Segmenten).
   5. **Display-an-Erkennung**: Verfahren *Ziffern erkannt* wählen (Mindestanzahl gültiger Ziffern, z. B. 5 von 7). Für die automatische Kalibrierung bei **ausgeschalteter** Anzeige **⏸ Hold** drücken und *Anzeige ist AUS* klicken; *Weiter*, dann bei sichtbaren Ziffern wieder Hold und *Anzeige ist AN*. Von jedem Zustand mindestens ein Bild (besser 3+, bei unterschiedlichem Licht), dann **🎯 Erkennung „Display an“ kalibrieren**. Ergebnis und Trennreserve werden angezeigt; einzelne Muster lassen sich löschen oder alle zurücksetzen.
   6. **💾 Speichern** (die Meldung im Kopf zeigt, ob nur live übernommen oder gespeichert wurde).
4. **MQTT** ist optional; Topics mit einem MQTT-Client prüfen (siehe unten).
5. **Seite 4 · Diagnose**, wenn etwas nicht funktioniert.

## MQTT-Topics

`<base>` ist das *Basis-Topic* (Standard `zaehlercam`).

| Topic | Inhalt |
|---|---|
| `<base>/value` | Zählerstand als Zahl mit eingestellten Nachkommastellen (Retain optional) |
| `<base>/raw` | erkannte Ziffern ohne Dezimalpunkt |
| `<base>/json` | `{"value":…, "raw":"…", "valid":true, "reconstructed":false, "uptime":…}`; bei unvollständigen Lesungen (Option) `{"value":null, "raw":"00?2123", "valid":false, …}` |
| `<base>/display` | `ON` / `OFF` – Anzeige erkannt an/aus |
| `<base>/rejected` | vollständige Lesung, die die Plausibilitätsprüfung verworfen hat (Ziffern als Text) |
| `<base>/seen` | nur wenn die Anzeige an war, aber **kein gültiger Wert** gelesen wurde: zuletzt gesehener Text, z. B. `00?2123` |
| `<base>/status` | `online` / `offline` (Last Will, retained) |

Optionen: *Immer senden* (jeden Wert) oder nur bei Änderung; *Retain* für value/raw.

## Plausibilitätsprüfung

Ein Wert wird nur als `value` gesendet, wenn **alle** Ziffern als gültige Muster gelesen wurden. Werte mit `?` gehen deshalb nie auf `value` – `00?2123` als Zahl zu parsen würde stillschweigend einen falschen Wert ergeben. Zusätzlich (Seite 1, *Plausibilitätsprüfung*, standardmäßig an):

- **Zähler darf nur hochzählen** – ein kleinerer Wert als der zuletzt akzeptierte wird verworfen und stattdessen auf `<base>/rejected` gesendet.
- **Max. Anstieg je Wert** (Standard 100 Zählschritte, `0` = unbegrenzt) – größere Sprünge (typisch: eine falsch gelesene hohe Ziffer) werden verworfen.
- **Bestätigungen für neue Basis** (Standard 3) – wird derselbe verworfene Wert so oft in Folge gelesen, gilt er als neue Basis (Zähler getauscht oder erster Wert nach dem Einschalten war falsch).
- **Fehlende Ziffern aus letztem Wert ergänzen** (Standard an, braucht max. Anstieg > 0) – bei einer Teil-Lesung wie `00?2195` wird die unleserliche Ziffer aus dem letzten Wert übernommen (`0022190` → `0022195`). Das Ergebnis wird nur verwendet, wenn es die obigen Prüfungen besteht, und im JSON mit `"reconstructed":true` gekennzeichnet.
- **Unvollständige Werte (mit ?) senden** (Standard aus) – wenn nichts ergänzt werden konnte, wird der Teiltext auf `<base>/raw` und `<base>/json` mit `"valid":false` gesendet, **nie auf `<base>/value`**. Nützlich zur Fehlersuche, nicht für Automatisierung.

Der zuletzt akzeptierte Wert liegt nur im RAM: Nach einem Neustart wird die erste vollständige Lesung ohne Vergleich akzeptiert. Verglichen wird als Ganzzahl der Ziffern, Nachkommastellen spielen keine Rolle.

## Ablauf einer Messung

`AUS → Display an erkannt → (Einschwingzeit, entfällt beim Verfahren „Ziffern erkannt“) → N Messungen (Mehrheit) → senden → warten bis Display aus`. Eine Messung zählt nur, wenn **alle** Ziffern gültige Muster ergeben. Optional wird bei dauerhaft eingeschalteter Anzeige alle *x* Sekunden neu gelesen. Geht die Anzeige vor Erreichen der Messzahl aus, wird die beste vorhandene Mehrheit gesendet.

## Verfahren der Display-an-Erkennung

| Verfahren | Löst aus, wenn | Geeignet bei |
|---|---|---|
| Helligkeit ≥ / ≤ Wert | mittlere Helligkeit im Rahmen über/unter einem Wert | stabilem Licht |
| Kontrast ≥ Wert | Standardabweichung im Rahmen | starkem Ziffernkontrast |
| Helligkeits-/Kontrast-Sprung (adaptiv) | Sprung gegenüber langsam nachgeführtem Ruhewert | driftendem Licht |
| **Ziffern erkannt** (empfohlen) | mindestens *N* Ziffern ergeben ein gültiges Muster | unabhängig von der Belichtung; braucht ein gutes Raster |

## Filter

*Hintergrund-Ausgleich* (Standard) vergleicht jeden Pixel mit dem lokalen Mittelwert (Radius in px); ein Segment gilt als an, wenn ≥ *x* % seiner Pixel um mehr als eine Schwelle (automatisch oder manuell) dunkler als die Umgebung sind. *Lokaler Kontrast* vergleicht Segmentstreifen mit ihren Nachbarn; *globale Schwelle* nutzt Otsu oder einen festen Wert. *Invertieren* = dunkle Segmente auf hellem Grund (typisches LCD).

## openHAB

Fertige Beispiele liegen in [`examples/openhab/`](examples/openhab/): MQTT-Things/Channels (inkl. Verfügbarkeit über den Last Will und JSON-Feld-Channels), Items, Persistence für rrd4j / mapdb / InfluxDB (`.persist`) und Regeln (Verbrauch 24 h / 7 Tage, Watchdog, Log verworfener Werte). Sie entstanden anhand der openHAB-Dokumentation und wurden nicht an einer laufenden Installation getestet.

## Diagnose

- **Bilder-Paket**: lädt mehrere Dateien auf einmal herunter – Einstellungen (ohne Passwörter), Messwerte als JSON, Livebild und das letzte „Display an“-Bild, jeweils mit und ohne Einzeichnung.
- **Aufnahme**: nimmt *n* Sekunden so schnell wie möglich auf – Video (`.webm` mit Einblendung) und/oder ZIP aus Einzel-JPEGs mit `messwerte.csv` (pro Bild: Display an/aus, Helligkeit, Kontrast, Schwelle, gültige Ziffern, erkannter Text, Segmentmasken). Der Browser holt die Bilder über WLAN, typisch sind ~1–8 Bilder/s; die *Firmware selbst* wertet etwa alle 120 ms aus.

## Fehlersuche

| Symptom | Wahrscheinliche Ursache / Abhilfe |
|---|---|
| Werte erscheinen auf `<base>/rejected` | die Plausibilitätsprüfung hat sie verworfen (kleiner als letzter Wert / Sprung zu groß); eine falsche Basis wird nach der eingestellten Zahl identischer Lesungen ersetzt, oder *Max. Anstieg* erhöhen |
| Kein Wert per MQTT, Status/Display kommen an | kein Bild mit **allen** Ziffern gültig → `<base>/seen` und die Log-Zeile `Display war an, aber KEIN gueltiger Wert…` ansehen; Raster/Filter korrigieren, *Fehlertoleranz* auf 1 |
| Ziffern `?` | Raster verschoben; schwaches Segment (oft der dünne obere Balken `a`); Toleranz 1, manuelle Schwelle senken (z. B. 4), auf scharfem Bild neu kalibrieren |
| Auto-Kalibrierung scheitert | Bild unscharf oder Spiegelung → Objektiv fokussieren, LED-Helligkeit senken, Rahmen enger um die Ziffern, bei eingeschalteter Anzeige kalibrieren |
| Display wird nie „an“ | Verfahren *Ziffern erkannt* nutzen; im gehaltenen AN-Bild müssen mindestens *N* Ziffern gelesen werden |
| Oberfläche träge | schwaches Netzteil oder schlechtes WLAN; andere Tabs mit Livebild schließen |
| LED-Ring lässt sich nicht ausschalten | falscher LED-Typ (der Ring ist WS2812B, nicht PWM) |
| OTA zeigt alte Version | falsche `.bin` hochgeladen; Seite hart neu laden, Version/Build vergleichen |

## Sicherheit

Die Weboberfläche hat **keine Authentifizierung**. Nur in einem vertrauenswürdigen Netz betreiben. Der *Export* schreibt WLAN-/MQTT-Passwörter in die JSON-Datei – exportierte Dateien nicht veröffentlichen (siehe `.gitignore`).

## Bekannte Einschränkungen

Nur 7-Segment-LCDs (bis 8 Ziffern); Perspektivverzerrung (Trapez) wird nicht korrigiert, nur Drehung; sehr unscharfe oder von Spiegelungen dominierte Bilder sind nicht lesbar; das serielle Log ist deutsch; der Text in den Diagnose-Einblendungen ist deutsch.

## Repository-Struktur

```
ZaehlerCam/ZaehlerCam.ino     Firmware + eingebettete Weboberfläche (eine Datei)
examples/                     Beispiel-Einstellungen des Autors (nur Rahmen/Raster/Filter/Erkennung)
examples/openhab/             openHAB Things, Items, Persistence, Regeln
README.md, README.de.md       Dokumentation
```

Die Beispiel-Einstellungen passen zu einer Kameraposition und einem Zähler; sie dienen nur als Beispiel für die JSON-Struktur. Teil-JSON-Dateien lassen sich importieren – fehlende Schlüssel behalten ihren Wert.

## Lizenz

Es wurde noch keine Lizenz gewählt – vor der Veröffentlichung eine `LICENSE`-Datei (z. B. MIT) hinzufügen.
