# openHAB examples / openHAB-Beispiele

| File | Purpose |
|---|---|
| `things/zaehlercam.things` | MQTT bridge + generic MQTT thing with channels (value, raw, display, seen, rejected, uptime, JSON fields), availability via the last will |
| `items/zaehlercam.items` | items linked to the channels, plus derived items (24 h / 7 d consumption, stale flag) |
| `persistence/rrd4j.persist` | charts |
| `persistence/mapdb.persist` | restore last value after restart |
| `persistence/influxdb.persist` | long-term history incl. rejected values |
| `rules/zaehlercam.rules` | consumption, watchdog, log of rejected values |

Copy the files into your openHAB `conf/` folders and adapt broker address, credentials and the topic prefix (`zaehlercam` = *base topic* in the ZaehlerCam web UI). The channel/item UIDs assume the bridge id `mosquitto` and the thing id `zaehlercam` as written in the `.things` file.

*Hints*
- Items with `Number:Energy` need the channel parameter `unit="kWh"` (already set). Meters in other units (m³, MWh) → change the unit in the channel and item.
- The ZaehlerCam only sends `/value` after a **plausible, fully read** value (see the plausibility check in the main README), so no extra openHAB-side filtering is required. The `rejected` / `seen` channels are for diagnostics.
- With *Retain* enabled, openHAB gets the last value immediately after a restart.
- The examples were written from the openHAB documentation and **have not been run against an openHAB installation** – please report problems.

*Deutsch:* Dateien in `conf/` kopieren; Broker, Zugangsdaten und Topic-Präfix anpassen. `/value` enthält nur plausible, vollständig gelesene Werte; `rejected`/`seen` dienen der Diagnose. Die Beispiele wurden **nicht** an einer laufenden openHAB-Instanz getestet.
