/*
  ZaehlerCam – ESP32-CAM meter reader for blinking 7-segment LCDs
  Copyright (C) 2026 the ZaehlerCam authors

  This program is free software: you can redistribute it and/or modify it under the terms of the
  GNU General Public License as published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version. It is distributed WITHOUT ANY WARRANTY; without
  even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License (file LICENSE) for more details.
*/
/*
  ZaehlerCam – ESP32-CAM liest eine 7-Segment-LCD-Anzeige aus und sendet den Wert per MQTT.

  Board:      "AI Thinker ESP32-CAM", PSRAM: Enabled
  Partition:  "Minimal SPIFFS (1.9MB APP with OTA)"  (oder Default 4MB with spiffs)
  Core:       arduino-esp32 2.0.x oder 3.x
  Libraries:  ArduinoJson 7.x, PubSubClient, Adafruit NeoPixel (alle über den Library-Manager)

  Erstes Flashen per USB/Seriell. Danach: http://<ip>/ -> "System" -> Firmware-Update (.bin)
  (Arduino IDE: Sketch -> Kompilierte Binärdatei exportieren).
  Ohne WLAN-Daten startet ein Access Point "ZaehlerCam-Setup" (http://192.168.4.1).
*/

#include <WiFi.h>
#include "esp_http_server.h"
#include "freertos/semphr.h"
#include <ESPmDNS.h>
#include <Update.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include "esp_camera.h"
#include "img_converters.h"

#define FW_VERSION "1.9.0"
#define MAXD 8
#define LED_CH 4          // LEDC-Kanal (nur Core 2.x), 0 ist von der Kamera belegt

// ---------- Kamera-Pins (AI-Thinker) ----------
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// Pins, die beim AI-Thinker frei nutzbar sind (4 = Blitz-LED, 33 = rote LED, 12/2/15 = Strapping-Pins!)
const int LED_PINS_OK[] = {2, 4, 12, 13, 14, 15, 33};

// ---------- Kameraparameter (Name, min, max, default) ----------
struct CamP { const char* n; int mn, mx, df; };
const CamP CP[] = {
  {"preview_quality", 5, 95, 60},   // nur JPEG-Vorschau im Browser
  {"brightness", -2, 2, 0}, {"contrast", -2, 2, 0}, {"saturation", -2, 2, 0},
  {"sharpness", -2, 2, 0}, {"denoise", 0, 8, 0}, {"special_effect", 0, 6, 0},
  {"whitebal", 0, 1, 1}, {"awb_gain", 0, 1, 1}, {"wb_mode", 0, 4, 0},
  {"exposure_ctrl", 0, 1, 1}, {"aec2", 0, 1, 0}, {"ae_level", -2, 2, 0},
  {"aec_value", 0, 1200, 300}, {"gain_ctrl", 0, 1, 1}, {"agc_gain", 0, 30, 0},
  {"gainceiling", 0, 6, 0}, {"bpc", 0, 1, 0}, {"wpc", 0, 1, 1}, {"raw_gma", 0, 1, 1},
  {"lenc", 0, 1, 1}, {"hmirror", 0, 1, 0}, {"vflip", 0, 1, 0}, {"dcw", 0, 1, 1}
};
const int NCAM = sizeof(CP) / sizeof(CP[0]);

const char* FSN[] = {"QVGA", "VGA", "SVGA", "XGA", "SXGA"};
const framesize_t FSV[] = {FRAMESIZE_QVGA, FRAMESIZE_VGA, FRAMESIZE_SVGA, FRAMESIZE_XGA, FRAMESIZE_SXGA};

// ---------- Konfiguration ----------
struct Rect { int x, y, w, h; };
struct Frm { float cx, cy, c, s; int dw, dh; };   // Display-Rahmen (Mitte, Groesse, Drehung); hier oben wegen Arduino-Prototypen   // Promille (0..1000)
struct Cfg {
  String ssid, wpass, host;
  bool mqEn, mqRetain, mqAlways;
  bool plEn, plFill, mqPartial; int plMax, plConf;   // Plausibilitaet: nur hochzaehlen, max. Sprung, Bestaetigungen, fehlende Ziffern aus letztem Wert ergaenzen, Teilwerte (mit ?) senden
  String mqHost, mqUser, mqPass, mqBase, mqCid;
  int mqPort;
  bool ledEn; int ledPin, ledPct, ledFreq, ledType, ledCount; String ledCol;   // ledType: 0 = PWM, 1 = WS2812
  bool camEn;
  String fs; int cam[NCAM];
  Rect disp;            // Display-Rahmen relativ zum Kamerabild
  float rot;            // Drehung des Rahmens in Grad (um dessen Mitte)
  bool lock;            // Rahmen/Raster gesperrt (nur Weboberflaeche, wird mitgespeichert)
  int n, dec;           // Anzahl Ziffern, Nachkommastellen
  int lay[6];           // Auto-Layout-Parameter (nur für die UI)
  bool segOk;
  String lang;          // Sprache der Weboberflaeche: "" = automatisch (Browser), "de" oder "en"
  Rect seg[MAXD][7];    // Segmente relativ zum Display-Rahmen
  uint8_t pat[10];      // Segmentmuster Ziffern 0-9 (Bit0=a ... Bit6=g)
  bool fAuto, fInv; int fThr, fPct, fTol, fMode, fDelta, fRad;   // fMode: 0 = globale Schwelle, 1 = lokaler Kontrast, 2 = Hintergrund-Ausgleich (adaptiv)
  int dMode, dLvl, dSettle, dSamples, dInt, dReeval;
} cfg;

void cfgDefaults() {
  cfg.ssid = ""; cfg.wpass = ""; cfg.host = "zaehlercam";
  cfg.mqEn = false; cfg.mqRetain = true; cfg.mqAlways = true;
  cfg.plEn = true; cfg.plFill = true; cfg.mqPartial = false; cfg.plMax = 100; cfg.plConf = 3;
  cfg.mqHost = ""; cfg.mqUser = ""; cfg.mqPass = ""; cfg.mqBase = "zaehlercam"; cfg.mqCid = "zaehlercam"; cfg.mqPort = 1883;
  cfg.ledEn = true; cfg.ledPin = 12; cfg.ledPct = 40; cfg.ledFreq = 20000;
  cfg.ledType = 1; cfg.ledCount = 8; cfg.ledCol = "#ffffff";
  cfg.fs = "VGA"; cfg.camEn = false;   // Kamera standardmaessig AUS
  for (int i = 0; i < NCAM; i++) cfg.cam[i] = CP[i].df;
  cfg.disp = {300, 350, 400, 300}; cfg.rot = 0; cfg.lock = false;
  cfg.n = 4; cfg.dec = 0;
  int lay[6] = {40, 130, 80, 20, 40, 0};
  for (int i = 0; i < 6; i++) cfg.lay[i] = lay[i];
  cfg.segOk = false;
  memset(cfg.seg, 0, sizeof(cfg.seg));
  uint8_t pat[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
  for (int i = 0; i < 10; i++) cfg.pat[i] = pat[i];
  cfg.fAuto = true; cfg.fInv = true; cfg.fThr = 128; cfg.fPct = 50; cfg.fTol = 0; cfg.fMode = 2; cfg.fDelta = 5; cfg.fRad = 14;
  cfg.dMode = 5; cfg.dLvl = 5; cfg.dSettle = 300; cfg.dSamples = 3; cfg.dInt = 120; cfg.dReeval = 0;
}

template <typename T> void rd(JsonVariantConst v, T& dst) { if (!v.isNull()) dst = v.as<T>(); }

void cfgFromJson(JsonVariantConst j) {
  rd(j["lang"], cfg.lang);
  JsonVariantConst w = j["wifi"];
  rd(w["ssid"], cfg.ssid); rd(w["pass"], cfg.wpass); rd(w["host"], cfg.host);
  JsonVariantConst m = j["mqtt"];
  rd(m["en"], cfg.mqEn); rd(m["host"], cfg.mqHost); rd(m["port"], cfg.mqPort); rd(m["user"], cfg.mqUser);
  rd(m["pass"], cfg.mqPass); rd(m["base"], cfg.mqBase); rd(m["cid"], cfg.mqCid);
  rd(m["retain"], cfg.mqRetain); rd(m["always"], cfg.mqAlways);
  { JsonVariantConst q = j["plaus"]; rd(q["en"], cfg.plEn); rd(q["fill"], cfg.plFill); rd(q["partial"], cfg.mqPartial); rd(q["max"], cfg.plMax); rd(q["conf"], cfg.plConf);
    cfg.plMax = constrain(cfg.plMax, 0, 100000000); cfg.plConf = constrain(cfg.plConf, 1, 20); }
  JsonVariantConst l = j["led"];
  rd(l["en"], cfg.ledEn); rd(l["pin"], cfg.ledPin); rd(l["pct"], cfg.ledPct); rd(l["freq"], cfg.ledFreq);
  rd(l["type"], cfg.ledType); rd(l["count"], cfg.ledCount); rd(l["col"], cfg.ledCol);
  cfg.ledPct = constrain(cfg.ledPct, 0, 100); cfg.ledFreq = constrain(cfg.ledFreq, 100, 300000);
  JsonVariantConst c = j["cam"];
  rd(c["fs"], cfg.fs); rd(c["en"], cfg.camEn);
  for (int i = 0; i < NCAM; i++) {
    JsonVariantConst v = c[CP[i].n];
    if (!v.isNull()) cfg.cam[i] = constrain(v.as<int>(), CP[i].mn, CP[i].mx);
  }
  JsonVariantConst d = j["disp"];
  rd(d["x"], cfg.disp.x); rd(d["y"], cfg.disp.y); rd(d["w"], cfg.disp.w); rd(d["h"], cfg.disp.h);
  rd(d["rot"], cfg.rot); cfg.rot = constrain(cfg.rot, -45.0f, 45.0f); rd(d["lock"], cfg.lock);
  rd(d["n"], cfg.n); rd(d["dec"], cfg.dec);
  cfg.n = constrain(cfg.n, 1, MAXD); cfg.dec = constrain(cfg.dec, 0, cfg.n - 1);
  JsonArrayConst la = d["lay"];
  for (int i = 0; i < 6 && i < (int)la.size(); i++) cfg.lay[i] = la[i].as<int>();
  JsonArrayConst pa = d["pat"];
  for (int i = 0; i < 10 && i < (int)pa.size(); i++) cfg.pat[i] = pa[i].as<int>() & 0x7F;
  JsonArrayConst sg = d["seg"];
  if (sg.size() > 0) {
    cfg.segOk = true;
    for (int dd = 0; dd < MAXD && dd < (int)sg.size(); dd++) {
      JsonArrayConst a = sg[dd];
      for (int s = 0; s < 7 && s < (int)a.size(); s++) {
        JsonArrayConst r = a[s];
        cfg.seg[dd][s].x = r[0].as<int>(); cfg.seg[dd][s].y = r[1].as<int>();
        cfg.seg[dd][s].w = r[2].as<int>(); cfg.seg[dd][s].h = r[3].as<int>();
      }
    }
  }
  JsonVariantConst f = j["flt"];
  rd(f["auto"], cfg.fAuto); rd(f["inv"], cfg.fInv); rd(f["thr"], cfg.fThr); rd(f["pct"], cfg.fPct); rd(f["tol"], cfg.fTol); rd(f["mode"], cfg.fMode); rd(f["delta"], cfg.fDelta); rd(f["rad"], cfg.fRad); cfg.fRad = constrain(cfg.fRad, 3, 60);
  JsonVariantConst t = j["det"];
  rd(t["mode"], cfg.dMode); rd(t["lvl"], cfg.dLvl); rd(t["settle"], cfg.dSettle); rd(t["samples"], cfg.dSamples);
  rd(t["interval"], cfg.dInt); rd(t["reeval"], cfg.dReeval);
  cfg.dSamples = constrain(cfg.dSamples, 1, 16); cfg.dInt = constrain(cfg.dInt, 30, 5000);
}

void cfgToJson(JsonDocument& j) {
  j["lang"] = cfg.lang;
  JsonObject w = j["wifi"].to<JsonObject>();
  w["ssid"] = cfg.ssid; w["pass"] = cfg.wpass; w["host"] = cfg.host;
  JsonObject m = j["mqtt"].to<JsonObject>();
  m["en"] = cfg.mqEn; m["host"] = cfg.mqHost; m["port"] = cfg.mqPort; m["user"] = cfg.mqUser;
  m["pass"] = cfg.mqPass; m["base"] = cfg.mqBase; m["cid"] = cfg.mqCid; m["retain"] = cfg.mqRetain; m["always"] = cfg.mqAlways;
  { JsonObject q = j["plaus"].to<JsonObject>(); q["en"] = cfg.plEn; q["fill"] = cfg.plFill; q["partial"] = cfg.mqPartial; q["max"] = cfg.plMax; q["conf"] = cfg.plConf; }
  JsonObject l = j["led"].to<JsonObject>();
  l["en"] = cfg.ledEn; l["pin"] = cfg.ledPin; l["pct"] = cfg.ledPct; l["freq"] = cfg.ledFreq;
  l["type"] = cfg.ledType; l["count"] = cfg.ledCount; l["col"] = cfg.ledCol;
  JsonObject c = j["cam"].to<JsonObject>();
  c["fs"] = cfg.fs; c["en"] = cfg.camEn;
  for (int i = 0; i < NCAM; i++) c[CP[i].n] = cfg.cam[i];
  JsonObject d = j["disp"].to<JsonObject>();
  d["x"] = cfg.disp.x; d["y"] = cfg.disp.y; d["w"] = cfg.disp.w; d["h"] = cfg.disp.h; d["rot"] = cfg.rot; d["lock"] = cfg.lock;
  d["n"] = cfg.n; d["dec"] = cfg.dec;
  JsonArray la = d["lay"].to<JsonArray>(); for (int i = 0; i < 6; i++) la.add(cfg.lay[i]);
  JsonArray pa = d["pat"].to<JsonArray>(); for (int i = 0; i < 10; i++) pa.add(cfg.pat[i]);
  JsonArray sg = d["seg"].to<JsonArray>();
  if (cfg.segOk) for (int dd = 0; dd < cfg.n; dd++) {
    JsonArray a = sg.add<JsonArray>();
    for (int s = 0; s < 7; s++) {
      JsonArray r = a.add<JsonArray>();
      r.add(cfg.seg[dd][s].x); r.add(cfg.seg[dd][s].y); r.add(cfg.seg[dd][s].w); r.add(cfg.seg[dd][s].h);
    }
  }
  JsonObject f = j["flt"].to<JsonObject>();
  f["auto"] = cfg.fAuto; f["inv"] = cfg.fInv; f["thr"] = cfg.fThr; f["pct"] = cfg.fPct; f["tol"] = cfg.fTol; f["mode"] = cfg.fMode; f["delta"] = cfg.fDelta; f["rad"] = cfg.fRad;
  JsonObject t = j["det"].to<JsonObject>();
  t["mode"] = cfg.dMode; t["lvl"] = cfg.dLvl; t["settle"] = cfg.dSettle; t["samples"] = cfg.dSamples;
  t["interval"] = cfg.dInt; t["reeval"] = cfg.dReeval;
}

void loadCfg() {
  File f = LittleFS.open("/config.json", "r");
  if (!f) return;
  JsonDocument d;
  if (!deserializeJson(d, f)) cfgFromJson(d.as<JsonVariantConst>());
  f.close();
}
void saveCfg() {
  JsonDocument d; cfgToJson(d);
  File f = LittleFS.open("/config.json", "w");
  if (f) { serializeJson(d, f); f.close(); }
}

// ---------- Globals ----------
httpd_handle_t httpd = NULL;
SemaphoreHandle_t mtx;                         // schuetzt cfg, res und Kamerazugriff
volatile bool pendMq = false, mqUp = false, otaBusy = false;
#define LOCK() xSemaphoreTake(mtx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(mtx)

// ---------- Selbstueberwachung (Watchdog, Neustart-Grund) ----------
RTC_NOINIT_ATTR uint32_t rtcMagic;   // bleibt ueber einen Software-Neustart erhalten
RTC_NOINIT_ATTR uint32_t rtcWhy;     // 1 = Hauptschleife haengt, 2 = WLAN dauerhaft weg, 3 = zu wenig Heap
volatile uint32_t loopBeat = 0; uint32_t wifiReconn = 0; char rstTxt[96] = "";
const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "Einschalten"; case ESP_RST_SW: return "Software-Neustart"; case ESP_RST_PANIC: return "ABSTURZ (Panic/Exception)";
    case ESP_RST_INT_WDT: return "Interrupt-Watchdog"; case ESP_RST_TASK_WDT: return "Task-Watchdog"; case ESP_RST_WDT: return "Watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (Spannungseinbruch!)"; case ESP_RST_EXT: return "Reset-Taste"; case ESP_RST_DEEPSLEEP: return "Deep-Sleep"; default: return "unbekannt";
  }
}
const char* whyName(uint32_t w) { return w == 1 ? "Hauptschleife haengt" : w == 2 ? "WLAN dauerhaft weg" : w == 3 ? "zu wenig Heap" : "?"; }
void selfRestart(uint32_t why, const char* txt) {
  Serial.printf("[WD] Neustart wegen: %s (Heap %u, min %u)\n", txt, (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());
  rtcMagic = 0xC0FFEE42; rtcWhy = why; delay(200); ESP.restart();
}
void wdTask(void*) {   // eigener Task: laeuft auch, wenn die Hauptschleife am Mutex/der Kamera haengt
  int lowN = 0;
  for (;;) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    if (millis() - loopBeat > 60000) selfRestart(1, "Hauptschleife reagiert seit 60 s nicht (Mutex/Kamera haengt?)");
    if (ESP.getFreeHeap() < 12000) { if (++lowN >= 6) selfRestart(3, "freier Heap < 12 kB seit 30 s"); } else lowN = 0;
  }
}
WiFiClient net;
PubSubClient mq(net);
bool camOk = false, apMode = false;
String camFs = "";
int curLedPin = -1, curLedFreq = 0, curLedMode = 0, curLedCnt = 0;   // Mode: 0 = frei, 1 = PWM, 2 = WS2812
Adafruit_NeoPixel* strip = nullptr;

// ---------- LED-Ring (PWM) ----------
bool pinAllowed(int p) { for (int v : LED_PINS_OK) if (v == p) return true; return false; }

void ledRelease() {   // aktuellen LED-Treiber freigeben, Pin auf LOW
  if (curLedMode == 2 && strip) { strip->clear(); strip->show(); delete strip; strip = nullptr; }
  if (curLedMode == 1) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(curLedPin);
#else
    ledcDetachPin(curLedPin);
#endif
  }
  if (curLedPin >= 0) { pinMode(curLedPin, OUTPUT); digitalWrite(curLedPin, LOW); }
  curLedMode = 0; curLedPin = -1;
}

void ledApply() {   // nur unter LOCK() bzw. im Setup aufrufen
  static String lastDesc = "";
  int pin = cfg.ledPin;
  if (!pinAllowed(pin)) {
    Serial.printf("[LED] Pin %d NICHT erlaubt (erlaubt: 2, 4, 12, 13, 14, 15, 33) - ignoriert, aktiver Pin bleibt %d\n", pin, curLedPin);
    if (curLedPin < 0) return;
    pin = curLedPin;
  }
  int mode = (cfg.ledType == 1) ? 2 : 1;
  int cnt = constrain(cfg.ledCount, 1, 64);
  if (curLedMode != 0 && (curLedMode != mode || curLedPin != pin || (mode == 2 && curLedCnt != cnt))) {
    Serial.printf("[LED] Treiber/Pin/Anzahl geaendert -> GPIO %d wird freigegeben\n", curLedPin);
    ledRelease();
  }
  char desc[128];
  if (mode == 2) {
    // WS2812B-Ring (z. B. WCMCU-2812B-8): serielles Datensignal, KEIN PWM
    if (!strip) {
      strip = new Adafruit_NeoPixel(cnt, pin, NEO_GRB + NEO_KHZ800);
      strip->begin();
      curLedMode = 2; curLedPin = pin; curLedCnt = cnt;
    }
    const char* cs = cfg.ledCol.c_str(); if (*cs == '#') cs++;
    uint32_t rgb = strtoul(cs, NULL, 16);
    int sc = cfg.ledEn ? cfg.ledPct : 0;
    uint8_t r = ((rgb >> 16) & 255) * sc / 100, g = ((rgb >> 8) & 255) * sc / 100, b = (rgb & 255) * sc / 100;
    for (int i = 0; i < cnt; i++) strip->setPixelColor(i, strip->Color(r, g, b));
    strip->show();
    snprintf(desc, sizeof(desc), "WS2812 GPIO %d, %d LEDs: %s, %d%%, Farbe %s -> RGB(%u,%u,%u)", pin, cnt,
             cfg.ledEn ? "AN" : "AUS", cfg.ledPct, cfg.ledCol.c_str(), r, g, b);
  } else {
    uint32_t duty = cfg.ledEn ? (cfg.ledPct * 255 + 50) / 100 : 0;
    if (curLedMode == 0) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      // fester Kanal 4 (Timer 2): Kamera belegt Kanal 0/Timer 0
      if (!ledcAttachChannel(pin, cfg.ledFreq, 8, LED_CH)) {
        Serial.printf("[LED] FEHLER: PWM auf GPIO %d konnte nicht gestartet werden\n", pin);
        return;
      }
#else
      ledcSetup(LED_CH, cfg.ledFreq, 8);
      ledcAttachPin(pin, LED_CH);
#endif
      curLedMode = 1; curLedPin = pin; curLedFreq = cfg.ledFreq;
    } else if (curLedFreq != cfg.ledFreq) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcChangeFrequency(curLedPin, cfg.ledFreq, 8);
#else
      ledcSetup(LED_CH, cfg.ledFreq, 8);
#endif
      curLedFreq = cfg.ledFreq;
    }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(curLedPin, duty);
    uint32_t rb = ledcRead(curLedPin);
#else
    ledcWrite(LED_CH, duty);
    uint32_t rb = ledcRead(LED_CH);
#endif
    snprintf(desc, sizeof(desc), "PWM GPIO %d: %s, %d%% -> Duty %u/255 (Readback %u), %d Hz", curLedPin,
             cfg.ledEn ? "AN" : "AUS", cfg.ledPct, (unsigned)duty, (unsigned)rb, cfg.ledFreq);
  }
  if (lastDesc != desc) { lastDesc = desc; Serial.printf("[LED] %s\n", desc); }
}

// ---------- Kamera ----------
void applyCam() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  for (int i = 0; i < NCAM; i++) {
    int v = cfg.cam[i]; const char* n = CP[i].n;
#define SET(name, fn) if (!strcmp(n, name) && s->fn) { s->fn(s, v); continue; }
    SET("brightness", set_brightness) SET("contrast", set_contrast) SET("saturation", set_saturation)
    SET("sharpness", set_sharpness) SET("denoise", set_denoise) SET("special_effect", set_special_effect)
    SET("whitebal", set_whitebal) SET("awb_gain", set_awb_gain) SET("wb_mode", set_wb_mode)
    SET("exposure_ctrl", set_exposure_ctrl) SET("aec2", set_aec2) SET("ae_level", set_ae_level)
    SET("aec_value", set_aec_value) SET("gain_ctrl", set_gain_ctrl) SET("agc_gain", set_agc_gain)
    SET("bpc", set_bpc) SET("wpc", set_wpc) SET("raw_gma", set_raw_gma) SET("lenc", set_lenc)
    SET("hmirror", set_hmirror) SET("vflip", set_vflip) SET("dcw", set_dcw)
    if (!strcmp(n, "gainceiling") && s->set_gainceiling) s->set_gainceiling(s, (gainceiling_t)v);
#undef SET
  }
}

bool camInit() {
  if (camOk) { esp_camera_deinit(); camOk = false; }
  framesize_t fsz = FRAMESIZE_VGA;
  for (int i = 0; i < 5; i++) if (cfg.fs == FSN[i]) fsz = FSV[i];
  camera_config_t c;
  memset(&c, 0, sizeof(c));
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;     // 8 Bit Graustufen -> direkt auswertbar
  c.frame_size = fsz;
  c.jpeg_quality = 12;
  if (psramFound()) {
    c.fb_count = 2; c.fb_location = CAMERA_FB_IN_PSRAM; c.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    c.fb_count = 1; c.fb_location = CAMERA_FB_IN_DRAM; c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    c.frame_size = FRAMESIZE_QVGA;
  }
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init FEHLGESCHLAGEN: 0x%x (%s) - Flachbandkabel, Stromversorgung, Board-Typ/Pins pruefen\n", err, esp_err_to_name(err));
    return false;
  }
  Serial.printf("[CAM] OK, Aufloesung %s, PSRAM %s, Framebuffer %d\n", cfg.fs.c_str(), psramFound() ? "ja" : "NEIN", (int)c.fb_count);
  camOk = true; camFs = cfg.fs;
  applyCam();
  return true;
}

// ---------- Bildauswertung ----------
struct Res {
  bool on, valid; float mean, sd, base; int thr, W, H, nvalid, nseg;
  uint8_t mask[MAXD]; uint8_t frac[MAXD][7]; char txt[MAXD + 3];   // frac = Anteil heller Pixel je Segment in %
} res;
// Zustand der adaptiven Display-an-Erkennung (global, damit eine Probe-Auswertung ihn zuruecksetzen kann)
float anBase = -1; int anLastMode = -1; uint32_t anTOn = 0;

// Display-Rahmen: Mitte + Groesse + Drehung. (u,v) = Pixel-Offset von der Rahmenmitte im "geraden" Rahmen.
Frm frmOf(int W, int H) {
  Frm f;
  f.dw = max(2, cfg.disp.w * W / 1000); f.dh = max(2, cfg.disp.h * H / 1000);
  f.cx = (cfg.disp.x + cfg.disp.w * 0.5f) * W / 1000.0f; f.cy = (cfg.disp.y + cfg.disp.h * 0.5f) * H / 1000.0f;
  float a = cfg.rot * 0.017453293f; f.c = cosf(a); f.s = sinf(a);
  return f;
}
static inline int smp(const uint8_t* p, int W, int H, const Frm& f, int u, int v) {
  int sx = (int)lroundf(f.cx + u * f.c - v * f.s), sy = (int)lroundf(f.cy + u * f.s + v * f.c);
  if (sx < 0) sx = 0; else if (sx >= W) sx = W - 1;
  if (sy < 0) sy = 0; else if (sy >= H) sy = H - 1;
  return p[(size_t)sy * W + sx];
}

// ---- Hintergrund-Ausgleich: "dunkler (bzw. heller) als die lokale Umgebung" ----
static uint32_t* integ = nullptr; static size_t integCap = 0; static int integW = 0, integH = 0, integR = 0; static bool integOk = false;
bool buildIntegral(const uint8_t* p, int W, int H) {
  size_t need = (size_t)(W + 1) * (H + 1);
  if (need > integCap) { if (integ) free(integ); integ = (uint32_t*)ps_malloc(need * sizeof(uint32_t)); integCap = integ ? need : 0; }
  if (!integ) { integOk = false; return false; }
  integW = W; integH = H;
  memset(integ, 0, (size_t)(W + 1) * sizeof(uint32_t));
  for (int y = 0; y < H; y++) {
    uint32_t row = 0; const uint8_t* r = p + (size_t)y * W; uint32_t* o = integ + (size_t)(y + 1) * (W + 1); const uint32_t* u = integ + (size_t)y * (W + 1);
    o[0] = 0;
    for (int x = 0; x < W; x++) { row += r[x]; o[x + 1] = u[x + 1] + row; }
  }
  integOk = true; return true;
}
static inline int bgAt(int x, int y, int R) {
  int x0 = x - R, x1 = x + R + 1, y0 = y - R, y1 = y + R + 1;
  if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0; if (x1 > integW) x1 = integW; if (y1 > integH) y1 = integH;
  const int W1 = integW + 1;
  uint32_t sum = integ[(size_t)y1 * W1 + x1] - integ[(size_t)y0 * W1 + x1] - integ[(size_t)y1 * W1 + x0] + integ[(size_t)y0 * W1 + x0];
  return (int)(sum / (uint32_t)((x1 - x0) * (y1 - y0)));
}
// Abweichung vom lokalen Mittel an der Quellposition zu (u,v); positiv = "Segment-Farbe" (dunkel bei fInv)
static inline int devAt(const uint8_t* p, int W, int H, const Frm& f, int u, int v, int R, bool inv) {
  int sx = (int)lroundf(f.cx + u * f.c - v * f.s), sy = (int)lroundf(f.cy + u * f.s + v * f.c);
  if (sx < 0) sx = 0; else if (sx >= W) sx = W - 1;
  if (sy < 0) sy = 0; else if (sy >= H) sy = H - 1;
  int b = bgAt(sx, sy, R), val = p[(size_t)sy * W + sx];
  return inv ? (b - val) : (val - b);
}

int otsu(const uint32_t* h, uint32_t tot) {
  double sumAll = 0; for (int i = 0; i < 256; i++) sumAll += (double)i * h[i];
  double sB = 0, wB = 0, best = -1; int th = 128;
  for (int t = 0; t < 256; t++) {
    wB += h[t]; if (wB == 0) continue;
    double wF = tot - wB; if (wF <= 0) break;
    sB += (double)t * h[t];
    double mB = sB / wB, mF = (sumAll - sB) / wF;
    double v = wB * wF * (mB - mF) * (mB - mF);
    if (v > best) { best = v; th = t; }
  }
  return th;
}

char matchDigit(uint8_t m) {
  if (m == 0) return ' ';
  int best = 99; char ch = '?';
  for (int k = 0; k < 10; k++) {
    int d = __builtin_popcount((m ^ cfg.pat[k]) & 0x7F);
    if (d < best) { best = d; ch = '0' + k; }
  }
  return best <= cfg.fTol ? ch : '?';
}

// Lokaler Kontrast: Segment-Mittelwert gegen die direkt angrenzenden Streifen
// (links/rechts bei senkrechten, oben/unten bei waagrechten Segmenten) -> unempfindlich gegen Helligkeitsverlauf
int segContrast(const uint8_t* p, int W, int H, const Frm& f, int u0, int v0, int sw, int sh, bool inv) {
  uint32_t in = 0, out = 0, nout = 0;
  for (int v = v0; v < v0 + sh; v++) for (int u = u0; u < u0 + sw; u++) in += smp(p, W, H, f, u, v);
  float mi = (float)in / (sw * sh);
  int b = (sh > sw) ? sw : sh; if (b < 2) b = 2;
  if (sh > sw) {
    for (int v = v0; v < v0 + sh; v++) for (int i = 1; i <= b; i++) { out += smp(p, W, H, f, u0 - i, v) + smp(p, W, H, f, u0 + sw - 1 + i, v); nout += 2; }
  } else {
    for (int i = 1; i <= b; i++) for (int u = u0; u < u0 + sw; u++) { out += smp(p, W, H, f, u, v0 - i) + smp(p, W, H, f, u, v0 + sh - 1 + i); nout += 2; }
  }
  if (!nout) return 0;
  float mo = (float)out / nout;
  return (int)lroundf(inv ? (mo - mi) : (mi - mo));
}

void analyze(camera_fb_t* fb) {
  const int W = fb->width, H = fb->height; const uint8_t* p = fb->buf;
  res.W = W; res.H = H;
  const Frm f = frmOf(W, H); const int dw = f.dw, dh = f.dh;

  uint32_t hist[256]; memset(hist, 0, sizeof(hist));
  uint64_t sum = 0, sq = 0; uint32_t n = 0; int st = (dw * dh > 60000) ? 2 : 1;
  for (int v = -dh / 2; v < dh - dh / 2; v += st)
    for (int u = -dw / 2; u < dw - dw / 2; u += st) { uint8_t val = (uint8_t)smp(p, W, H, f, u, v); hist[val]++; sum += val; sq += (uint32_t)val * val; n++; }
  if (!n) return;
  res.mean = (float)sum / n;
  float var = (float)sq / n - res.mean * res.mean; res.sd = var > 0 ? sqrtf(var) : 0;

  // Display an/aus
  if (cfg.dMode != anLastMode) { anLastMode = cfg.dMode; anBase = -1; }
  if (cfg.dMode == 3 || cfg.dMode == 4) {
    // adaptiv: Sprung gegenueber einem langsam nachgefuehrten Ruhewert (3 = Helligkeit, 4 = Kontrast)
    float m = (cfg.dMode == 4) ? res.sd : res.mean;
    if (anBase < 0) anBase = m;
    float jump = m - anBase; bool was = res.on;
    res.on = was ? (jump >= cfg.dLvl - 4) : (jump >= cfg.dLvl);
    if (res.on && !was) anTOn = millis();
    if (res.on && millis() - anTOn > 15000) { anBase = m; res.on = false; }   // dauerhaft hoeher -> neuer Ruhewert
    if (!res.on) anBase += (m - anBase) * 0.05f;
    res.base = anBase;
  } else if (cfg.dMode == 5) {
    res.base = 0;   // Entscheidung erst nach der Ziffernerkennung (siehe unten)
  } else {
    // absolut (mit Hysterese 5)
    float m = (cfg.dMode == 2) ? res.sd : res.mean;
    if (cfg.dMode == 1) res.on = res.on ? (m <= cfg.dLvl + 5) : (m <= cfg.dLvl);
    else                res.on = res.on ? (m >= cfg.dLvl - 5) : (m >= cfg.dLvl);
    res.base = 0;
  }

  const bool m2 = (cfg.fMode == 2) && buildIntegral(p, W, H);
  int rad = max(3, cfg.fRad * W / 640); integR = rad;
  int T2 = 8;
  if (m2) {
    if (cfg.fAuto) {
      uint32_t dh2[64]; memset(dh2, 0, sizeof(dh2)); uint32_t nn = 0;
      for (int v = -dh / 2; v < dh - dh / 2; v += 2) for (int u = -dw / 2; u < dw - dw / 2; u += 2) { int d2 = devAt(p, W, H, f, u, v, rad, cfg.fInv); if (d2 < 0) d2 = 0; if (d2 > 63) d2 = 63; dh2[d2]++; nn++; }
      uint32_t acc = 0, lim = nn / 100; int p99 = 63; for (int i = 63; i >= 0; i--) { acc += dh2[i]; if (acc > lim) { p99 = i; break; } }
      T2 = constrain((int)lroundf(0.2f * p99), 4, 14);
    } else T2 = constrain(cfg.fThr, 1, 60);
    res.thr = T2;
  } else {
    res.thr = cfg.fAuto ? otsu(hist, n) : cfg.fThr;
  }

  // Segmente
  char chars[MAXD]; bool ok = cfg.segOk, any = false;
  for (int d = 0; d < cfg.n; d++) {
    uint8_t mask = 0;
    if (cfg.segOk) for (int s = 0; s < 7; s++) {
      const Rect& r = cfg.seg[d][s];
      int u0 = r.x * dw / 1000 - dw / 2, v0 = r.y * dh / 1000 - dh / 2;
      int sw = max(1, r.w * dw / 1000), sh = max(1, r.h * dh / 1000);
      if (m2) {
        uint32_t lit2 = 0;
        for (int v = v0; v < v0 + sh; v++) for (int u = u0; u < u0 + sw; u++) if (devAt(p, W, H, f, u, v, rad, cfg.fInv) > T2) lit2++;
        int fr2 = (int)(lit2 * 100 / (sw * sh)); res.frac[d][s] = fr2;
        if (fr2 >= cfg.fPct) mask |= (1 << s);
        continue;
      }
      if (cfg.fMode == 1) {
        int c = segContrast(p, W, H, f, u0, v0, sw, sh, cfg.fInv);
        res.frac[d][s] = (uint8_t)constrain(c, 0, 100);
        if (c >= cfg.fDelta) mask |= (1 << s);
        continue;
      }
      uint32_t lit = 0;
      for (int v = v0; v < v0 + sh; v++)
        for (int u = u0; u < u0 + sw; u++) { int val = smp(p, W, H, f, u, v); if (cfg.fInv ? (val < res.thr) : (val > res.thr)) lit++; }
      int fr = (int)(lit * 100 / (sw * sh)); res.frac[d][s] = fr;
      if (fr >= cfg.fPct) mask |= (1 << s);
    }
    res.mask[d] = mask;
    chars[d] = matchDigit(mask);
    if (chars[d] == '?') ok = false;
    if (chars[d] != ' ') any = true;
  }
  if (cfg.dMode == 5) {
    // Display gilt als "an", wenn genug gueltige Ziffern erkannt werden (unabhaengig von Belichtung/Helligkeit)
    int nvalid = 0;
    for (int d = 0; d < cfg.n; d++) if (chars[d] != ' ' && chars[d] != '?') nvalid++;
    int need = cfg.dLvl < 1 ? 1 : cfg.dLvl; if (need > cfg.n) need = cfg.n;
    res.on = res.on ? (nvalid >= (need > 1 ? need - 1 : 1)) : (nvalid >= need);
  }
  int k = 0;
  for (int d = 0; d < cfg.n; d++) {
    if (cfg.dec > 0 && d == cfg.n - cfg.dec) res.txt[k++] = '.';
    res.txt[k++] = chars[d];
  }
  res.txt[k] = 0;
  { int nv2 = 0, ns2 = 0; for (int d = 0; d < cfg.n; d++) { if (chars[d] != ' ' && chars[d] != '?') nv2++; ns2 += __builtin_popcount(res.mask[d]); } res.nvalid = nv2; res.nseg = ns2; }
  res.valid = res.on && ok && any;
}

// Binär-Vorschau: Display-Bereich als Schwarz/Weiß, Rest abgedunkelt
void filterPreview(camera_fb_t* fb) {
  int W = fb->width, H = fb->height; const Frm f = frmOf(W, H);
  const bool m2 = (cfg.fMode == 2) && buildIntegral(fb->buf, W, H);
  if (m2) {
    // Ergebnisbild in einen Puffer schreiben, weil die Umgebung aus dem Originalbild stammt (nicht in-place)
    uint8_t* o = (uint8_t*)ps_malloc((size_t)W * H);
    if (o) {
      int rad = max(3, cfg.fRad * W / 640), T2 = (res.thr > 0) ? res.thr : 8;
      for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
        float ox = x - f.cx, oy = y - f.cy, u = ox * f.c + oy * f.s, v = -ox * f.s + oy * f.c;
        int val = fb->buf[(size_t)y * W + x];
        if (fabsf(u) < f.dw * 0.5f && fabsf(v) < f.dh * 0.5f) {
          int b = bgAt(x, y, rad), dv = cfg.fInv ? (b - val) : (val - b);
          o[(size_t)y * W + x] = (dv > T2) ? 0 : 255;   // Segmente schwarz auf weiss
        } else o[(size_t)y * W + x] = val >> 2;
      }
      memcpy(fb->buf, o, (size_t)W * H); free(o); return;
    }
  }
  for (int y = 0; y < H; y++) {
    uint8_t* r = fb->buf + (size_t)y * W;
    for (int x = 0; x < W; x++) {
      float ox = x - f.cx, oy = y - f.cy;
      float u = ox * f.c + oy * f.s, v = -ox * f.s + oy * f.c;   // zurueck in den geraden Rahmen
      uint8_t val = r[x];
      if (fabsf(u) < f.dw * 0.5f && fabsf(v) < f.dh * 0.5f) r[x] = (cfg.fInv ? (val < res.thr) : (val > res.thr)) ? 255 : 0;
      else r[x] = val >> 2;
    }
  }
}

// ---------- Diagnose: letztes Bild bei eingeschaltetem Display merken ----------
volatile uint32_t lastAnTs = 0;   // millis() der letzten Auswertung
uint8_t* keepJpg = nullptr; size_t keepLen = 0; uint32_t keepT = 0; Res keepRes; bool capDone = false;
void keepFrame(camera_fb_t* fb) {   // unter LOCK() aufrufen
  uint8_t* out = nullptr; size_t len = 0;
  if (!frame2jpg(fb, 85, &out, &len)) return;
  uint8_t* pp = (uint8_t*)ps_malloc(len);
  if (pp) { memcpy(pp, out, len); free(out); } else pp = out;
  if (keepJpg) free(keepJpg);
  keepJpg = pp; keepLen = len; keepT = millis(); keepRes = res;
  Serial.printf("[DIAG] Bild bei Display-AN gespeichert (%u Bytes)\n", (unsigned)len);
}

// ---------- MQTT ----------
char mqHostBuf[64] = "";
void mqReconf() {   // nur im Loop-Task (unter LOCK) oder im Setup
  mq.disconnect(); mqUp = false;
  strlcpy(mqHostBuf, cfg.mqHost.c_str(), sizeof(mqHostBuf));
  mq.setServer(mqHostBuf, cfg.mqPort); mq.setSocketTimeout(2);
  Serial.printf("[MQTT] %s, Broker '%s':%d, Basis-Topic '%s'\n", cfg.mqEn ? "aktiv" : "AUS", mqHostBuf, cfg.mqPort, cfg.mqBase.c_str());
}
void mqPub(const char* sub, const String& v, bool ret) {
  if (!cfg.mqEn || !mq.connected()) return;
  String t = cfg.mqBase + "/" + sub;
  mq.publish(t.c_str(), v.c_str(), ret);
}
void mqLoop() {
  if (pendMq) { pendMq = false; LOCK(); mqReconf(); UNLOCK(); }
  if (!cfg.mqEn || WiFi.status() != WL_CONNECTED || mqHostBuf[0] == 0) { mqUp = false; return; }
  if (mq.connected()) { mqUp = true; mq.loop(); return; }
  mqUp = false;
  static uint32_t t = 0;
  if (millis() - t < 5000) return;
  t = millis();
  String cid, user, pass, lwt;
  LOCK(); cid = cfg.mqCid; user = cfg.mqUser; pass = cfg.mqPass; lwt = cfg.mqBase + "/status"; UNLOCK();
  Serial.printf("[MQTT] verbinde mit %s:%d ...\n", mqHostBuf, cfg.mqPort);
  bool ok = user.length()
    ? mq.connect(cid.c_str(), user.c_str(), pass.c_str(), lwt.c_str(), 0, true, "offline")
    : mq.connect(cid.c_str(), NULL, NULL, lwt.c_str(), 0, true, "offline");
  if (ok) { mq.publish(lwt.c_str(), "online", true); mqUp = true; Serial.println("[MQTT] verbunden"); }
  else Serial.printf("[MQTT] Verbindung fehlgeschlagen, Status %d (-2 Broker nicht erreichbar, 4/5 Zugangsdaten falsch)\n", mq.state());
}

// ---------- Ablauf: Display geht an -> auswerten -> senden ----------
enum { ST_OFF, ST_SETTLE, ST_SAMPLE, ST_DONE };
int stt = ST_OFF; uint32_t tSt = 0; String votes[16]; int nv = 0;
String lastRaw = "", lastVal = "";
char lastOnTxt[MAXD + 3] = ""; int lastOnNv = 0; bool pubThis = false;

String cleanVal(const char* t) { String s; for (; *t; t++) if (*t != ' ') s += *t; return s; }

// ---------- Plausibilitaet ----------
long long lastAcc = -1; String lastAccDigits = "", lastRej = ""; long long rejVal = -1; int rejN = 0;
String bestPartial = "", lastValid = ""; int partN = 0;   // beste Teil-Lesung (mit ?) der aktuellen Einschaltphase

String digitsOf(const String& s2) { String o; for (size_t i = 0; i < s2.length(); i++) if (s2[i] != '.' && s2[i] != ' ') o += s2[i]; return o; }
String fmtVal(const String& digits) {   // Ziffern -> Zahl mit Nachkommastellen
  String t = digits; if (cfg.dec > 0 && (int)t.length() > cfg.dec) t = t.substring(0, t.length() - cfg.dec) + "." + t.substring(t.length() - cfg.dec);
  char vbuf[24]; snprintf(vbuf, sizeof(vbuf), "%.*f", cfg.dec, atof(t.c_str())); return String(vbuf);
}
// 0 = plausibel, 1 = kleiner als der letzte Wert, 2 = Sprung groesser als erlaubt
int plCheck(long long v) {
  if (!cfg.plEn || lastAcc < 0) return 0;
  if (v < lastAcc) return 1;
  if (cfg.plMax > 0 && v - lastAcc > cfg.plMax) return 2;
  return 0;
}
void pubValue(const String& digits, bool recon, int nbest, int nm) {
  String val = fmtVal(digits);
  bool changed = (val != lastVal); pubThis = true;
  lastRaw = digits; lastVal = val; lastAcc = atoll(digits.c_str()); lastAccDigits = digits;
  Serial.printf("[ZAEHLER] Wert %s (raw '%s', %d von %d Messungen gleich%s) %s\n", val.c_str(), digits.c_str(), nbest, nm, recon ? ", fehlende Ziffern aus letztem Wert ergaenzt" : "",
                (cfg.mqEn && (cfg.mqAlways || changed)) ? (mqUp ? "-> MQTT gesendet" : "-> MQTT NICHT verbunden, nicht gesendet") : "(kein MQTT-Versand)");
  if (cfg.mqEn && (cfg.mqAlways || changed)) {
    mqPub("value", val, cfg.mqRetain);
    mqPub("raw", digits, cfg.mqRetain);
    mqPub("json", "{\"value\":" + val + ",\"raw\":\"" + digits + "\",\"valid\":true,\"reconstructed\":" + (recon ? "true" : "false") + ",\"uptime\":" + String(millis() / 1000) + "}", false);
  }
}

void publishBest() {
  if (nv == 0) return;
  int bi = 0, bc = 0;
  for (int i = 0; i < nv; i++) { int c = 0; for (int j = 0; j < nv; j++) if (votes[i] == votes[j]) c++; if (c > bc) { bc = c; bi = i; } }
  String digits = digitsOf(votes[bi]); int nm = nv, nbest = bc; nv = 0;
  long long v = atoll(digits.c_str());
  int bad = plCheck(v);
  if (bad) {
    if (v == rejVal) rejN++; else { rejVal = v; rejN = 1; }
    if (rejN >= cfg.plConf) {
      Serial.printf("[PLAUS] Wert %s wurde %d-mal in Folge gelesen -> als neue Basis uebernommen (vorher %lld)\n", digits.c_str(), rejN, lastAcc);
      lastAcc = -1; rejN = 0; rejVal = -1;
    } else {
      lastRej = digits; pubThis = true;
      Serial.printf("[PLAUS] Wert %s VERWORFEN (%s; letzter gueltiger Wert %s, %d von %d Bestaetigungen)\n", digits.c_str(),
                    bad == 1 ? "kleiner als der letzte Wert" : "Sprung groesser als erlaubt", lastAccDigits.c_str(), rejN, cfg.plConf);
      if (cfg.mqEn) mqPub("rejected", digits, false);
      return;
    }
  } else { rejN = 0; rejVal = -1; }
  pubValue(digits, false, nbest, nm);
}

// Anzeige war an, aber keine vollstaendig gueltige Messung: Teil-Lesung ergaenzen oder melden. true = es wurde etwas veroeffentlicht/behandelt
bool handlePartial() {
  if ((int)bestPartial.length() != cfg.n || partN <= 0) return false;
  if (cfg.plEn && cfg.plFill && cfg.plMax > 0 && lastAcc >= 0 && (int)lastAccDigits.length() == cfg.n) {
    String f = bestPartial;
    for (int i = 0; i < cfg.n; i++) if (f[i] == '?') f[i] = lastAccDigits[i];
    long long v = atoll(f.c_str());
    if (plCheck(v) == 0) { pubValue(f, true, 1, 1); return true; }
    Serial.printf("[PLAUS] Teil-Lesung '%s' mit Ergaenzung %s nicht plausibel (letzter Wert %s)\n", bestPartial.c_str(), f.c_str(), lastAccDigits.c_str());
  }
  if (cfg.mqPartial && cfg.mqEn) {
    mqPub("raw", bestPartial, false);
    mqPub("json", "{\"value\":null,\"raw\":\"" + bestPartial + "\",\"valid\":false,\"uptime\":" + String(millis() / 1000) + "}", false);
    Serial.printf("[ZAEHLER] Teil-Lesung '%s' (unvollstaendig) gesendet (nur raw/json, nie value)\n", bestPartial.c_str());
    pubThis = true; return true;
  }
  return false;
}

void stepMachine() {
  uint32_t now = millis();
  if (!res.on) {
    if (stt == ST_SAMPLE && nv > 0) publishBest();
    if (stt != ST_OFF && !pubThis && lastValid.length()) {   // Anzeige war nur kurz an: gueltige Lesung aus der Einschwingphase nachtragen
      votes[0] = lastValid; nv = 1; Serial.println("[ZAEHLER] Anzeige war zu kurz an fuer die Messreihe - verwende die zuletzt gueltige Lesung"); publishBest();
    }
    if (stt != ST_OFF) {
      if (!pubThis && !handlePartial()) {
        Serial.printf("[ZAEHLER] Display war an, aber KEIN gueltiger Wert gelesen (zuletzt gesehen: '%s', %d von %d Ziffern gueltig) -> Raster/Filter/Toleranz pruefen\n", lastOnTxt, lastOnNv, cfg.n);
        mqPub("seen", String(lastOnTxt), false);
      }
      stt = ST_OFF; nv = 0; mqPub("display", "OFF", false); Serial.println("[DISP] Display AUS");
    }
    return;
  }
  strncpy(lastOnTxt, res.txt, sizeof(lastOnTxt) - 1); lastOnTxt[sizeof(lastOnTxt) - 1] = 0; lastOnNv = res.nvalid;
  if (!res.valid && res.nvalid > partN && res.nvalid >= (cfg.n + 1) / 2) {   // beste Teil-Lesung merken (Ziffern, fehlende/ungueltige = ?)
    String pc; for (const char* c = res.txt; *c; c++) { if (*c == '.') continue; pc += (*c == ' ') ? '?' : *c; }
    if ((int)pc.length() == cfg.n) { bestPartial = pc; partN = res.nvalid; }
  }
  if (res.valid) lastValid = String(res.txt);
  for (bool again = true; again;) {   // Zustandswechsel im selben Bild weiterverarbeiten, damit bei sehr kurzer Anzeige kein Bild verloren geht
    again = false;
    switch (stt) {
      case ST_OFF: stt = ST_SETTLE; tSt = now; nv = 0; pubThis = false; bestPartial = ""; partN = 0; lastValid = res.valid ? String(res.txt) : String(""); mqPub("display", "ON", false);
        Serial.printf("[DISP] Display AN (Helligkeit %.0f, Kontrast %.0f, Schwelle %d)\n", res.mean, res.sd, res.thr);
        if (cfg.dMode == 5) again = true;   // Erkennung beruht schon auf gelesenen Ziffern -> keine Einschwingzeit
        break;
      case ST_SETTLE: if (cfg.dMode == 5 || now - tSt >= (uint32_t)cfg.dSettle) { stt = ST_SAMPLE; tSt = now; nv = 0; again = true; } break;
      case ST_SAMPLE:
        if (res.valid && nv < 16) votes[nv++] = String(res.txt);
        if (nv >= cfg.dSamples) { publishBest(); stt = ST_DONE; tSt = now; }
        else if (now - tSt > 5000) { publishBest(); stt = ST_DONE; tSt = now; }
        break;
      case ST_DONE:
        if (cfg.dReeval > 0 && now - tSt > (uint32_t)cfg.dReeval * 1000UL) { stt = ST_SAMPLE; tSt = now; nv = 0; }
        break;
    }
  }
}

// ---------- Web ----------
extern const char PAGE[] PROGMEM;

void applyAll() {   // nur unter LOCK() aufrufen
  ledApply();
  pendMq = true;
  if (!cfg.camEn) {
    if (camOk) {
      esp_camera_deinit(); camOk = false; camFs = "";
      res.on = false; res.valid = false; res.txt[0] = 0; stt = ST_OFF; nv = 0;
      Serial.println("[CAM] ausgeschaltet");
    }
  } else if (!camOk || cfg.fs != camFs) {
    Serial.printf("[CAM] wird gestartet (Aufloesung %s)...\n", cfg.fs.c_str()); camInit();
  } else { applyCam(); Serial.println("[CAM] Einstellungen angewendet"); }
}

static esp_err_t sendTxt(httpd_req_t* r, int code, const char* s) {
  httpd_resp_set_status(r, code == 200 ? "200 OK" : code == 400 ? "400 Bad Request" : code == 503 ? "503 Service Unavailable" : code == 404 ? "404 Not Found" : "500 Internal Server Error");
  httpd_resp_set_type(r, "text/plain");
  return httpd_resp_send(r, s, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t sendJson(httpd_req_t* r, const String& s) {
  httpd_resp_set_type(r, "application/json");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  return httpd_resp_send(r, s.c_str(), s.length());
}
static bool readBody(httpd_req_t* r, String& out, size_t maxLen) {
  size_t len = r->content_len;
  if (len == 0 || len > maxLen) return false;
  char* b = (char*)malloc(len + 1);
  if (!b) return false;
  size_t got = 0; int to = 0;
  while (got < len) {
    int n = httpd_req_recv(r, b + got, len - got);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) { if (++to > 5) { free(b); return false; } continue; }
    if (n <= 0) { free(b); return false; }
    got += n;
  }
  b[len] = 0; out = String(b); free(b);
  return true;
}

static esp_err_t hPage(httpd_req_t* r) {
  httpd_resp_set_type(r, "text/html; charset=utf-8");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store, max-age=0");
  return httpd_resp_send(r, PAGE, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t hCfgGet(httpd_req_t* r) {
  JsonDocument d; String o;
  LOCK(); cfgToJson(d); UNLOCK();
  serializeJson(d, o);
  char q[48];
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK && strstr(q, "download"))
    httpd_resp_set_hdr(r, "Content-Disposition", "attachment; filename=zaehlercam-config.json");
  return sendJson(r, o);
}
static esp_err_t hCfgPost(httpd_req_t* r) {
  String body;
  if (!readBody(r, body, 16384)) return sendTxt(r, 400, "Body fehlt oder zu gross");
  JsonDocument d;
  if (deserializeJson(d, body)) return sendTxt(r, 400, "JSON ungueltig");
  char q[48] = "", v[4] = ""; bool save = false;
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK && httpd_query_key_value(q, "save", v, sizeof(v)) == ESP_OK) save = (v[0] == '1');
  LOCK();
  cfgFromJson(d.as<JsonVariantConst>());
  applyAll();
  if (save) saveCfg();
  UNLOCK();
  Serial.printf("[CFG] Einstellungen empfangen (%s)\n", save ? "GESPEICHERT" : "live, nicht gespeichert");
  if (save) Serial.printf("[CFG] WLAN-SSID='%s', Passwort-Laenge %d (WLAN-Aenderung gilt nach Neustart)\n", cfg.ssid.c_str(), cfg.wpass.length());
  return sendTxt(r, 200, "OK");
}
static esp_err_t hCamDef(httpd_req_t* r) {
  JsonDocument d; JsonArray a = d.to<JsonArray>();
  for (int i = 1; i < NCAM; i++) { JsonObject o = a.add<JsonObject>(); o["n"] = CP[i].n; o["mn"] = CP[i].mn; o["mx"] = CP[i].mx; }
  JsonObject q = a.add<JsonObject>(); q["n"] = CP[0].n; q["mn"] = CP[0].mn; q["mx"] = CP[0].mx;
  String o; serializeJson(d, o);
  return sendJson(r, o);
}
static esp_err_t hStatus(httpd_req_t* r) {
  JsonDocument d;
  LOCK();
  d["ip"] = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  d["rssi"] = WiFi.RSSI(); d["up"] = millis() / 1000; d["heap"] = ESP.getFreeHeap(); d["psram"] = ESP.getFreePsram();
  d["fw"] = FW_VERSION; d["build"] = __DATE__ " " __TIME__; d["cam"] = camOk; d["camEn"] = cfg.camEn; d["w"] = res.W; d["h"] = res.H;
  d["on"] = res.on; d["mean"] = res.mean; d["sd"] = res.sd; d["thr"] = res.thr; d["base"] = res.base;
  d["txt"] = res.txt; d["valid"] = res.valid; d["raw"] = lastRaw; d["val"] = lastVal; d["rej"] = lastRej; d["acc"] = lastAccDigits; d["minheap"] = ESP.getMinFreeHeap(); d["rst"] = rstTxt; d["wrc"] = wifiReconn; d["mq"] = (bool)mqUp;
  JsonArray m = d["mask"].to<JsonArray>(); JsonArray fq = d["frac"].to<JsonArray>();
  for (int i = 0; i < cfg.n; i++) { m.add(res.mask[i]); JsonArray fr = fq.add<JsonArray>(); for (int k = 0; k < 7; k++) fr.add(res.frac[i][k]); }
  UNLOCK();
  String o; serializeJson(d, o);
  return sendJson(r, o);
}
// Messwerte (aus res) als JSON fuer den Header X-Meta
static void buildMeta(char* meta, size_t sz, uint32_t age) {
  int k = snprintf(meta, sz, "{\"t\":%lu,\"age\":%lu,\"on\":%d,\"valid\":%d,\"mean\":%.1f,\"sd\":%.1f,\"thr\":%d,\"base\":%.1f,\"st\":%d,\"nv\":%d,\"ns\":%d,\"txt\":\"%s\",\"mask\":[",
    (unsigned long)millis(), (unsigned long)age, res.on ? 1 : 0, res.valid ? 1 : 0, res.mean, res.sd, res.thr, res.base, stt, res.nvalid, res.nseg, res.txt);
  for (int d = 0; d < cfg.n && k < (int)sz - 8; d++) k += snprintf(meta + k, sz - k, "%s%d", d ? "," : "", res.mask[d]);
  if (k < (int)sz - 3) snprintf(meta + k, sz - k, "]}");
}

static esp_err_t hSnap(httpd_req_t* r) {
  uint8_t* out = nullptr; size_t len = 0; bool ok = false, bin = false, probe = false;
  char q[64], qv[8]; int qual = cfg.cam[0];
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK) {
    if (strstr(q, "bin=1")) bin = true;
    if (httpd_query_key_value(q, "m", qv, sizeof(qv)) == ESP_OK) probe = (qv[0] == '1');
    if (httpd_query_key_value(q, "q", qv, sizeof(qv)) == ESP_OK) qual = constrain(atoi(qv), 10, 95);
  }
  char meta[320]; meta[0] = 0;
  LOCK();
  if (camOk) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      if (probe) {
        // Genau dieses Bild auswerten, ohne den Zustand der laufenden Erkennung zu veraendern
        Res save = res; float sb = anBase; int sl = anLastMode; uint32_t so = anTOn;
        analyze(fb);
        buildMeta(meta, sizeof(meta), 0);
        res = save; anBase = sb; anLastMode = sl; anTOn = so;
      } else {
        buildMeta(meta, sizeof(meta), millis() - lastAnTs);   // Messwerte der letzten Auswertung
      }
      if (bin) filterPreview(fb);
      ok = frame2jpg(fb, qual, &out, &len); esp_camera_fb_return(fb);
    }
  }
  UNLOCK();
  if (!ok) { if (cfg.camEn) Serial.println("[HTTP] Snapshot fehlgeschlagen (Kamera nicht bereit?)"); return sendTxt(r, 503, "Kein Bild"); }
  httpd_resp_set_type(r, "image/jpeg");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  if (meta[0]) httpd_resp_set_hdr(r, "X-Meta", meta);
  esp_err_t e = httpd_resp_send(r, (const char*)out, len);
  free(out);
  return e;
}
static esp_err_t hDiagInfo(httpd_req_t* r) {
  JsonDocument d;
  LOCK();
  d["have"] = (keepJpg != nullptr);
  if (keepJpg) {
    d["age"] = (millis() - keepT) / 1000;
    JsonObject o = d["res"].to<JsonObject>();
    o["mean"] = keepRes.mean; o["sd"] = keepRes.sd; o["thr"] = keepRes.thr; o["w"] = keepRes.W; o["h"] = keepRes.H;
    o["txt"] = keepRes.txt; o["valid"] = keepRes.valid;
    JsonArray m = o["mask"].to<JsonArray>(); JsonArray fq = o["frac"].to<JsonArray>();
    for (int i = 0; i < cfg.n; i++) { m.add(keepRes.mask[i]); JsonArray fr = fq.add<JsonArray>(); for (int k = 0; k < 7; k++) fr.add(keepRes.frac[i][k]); }
  }
  UNLOCK();
  String o; serializeJson(d, o);
  return sendJson(r, o);
}
static esp_err_t hDiagImage(httpd_req_t* r) {
  LOCK();
  if (!keepJpg) { UNLOCK(); return sendTxt(r, 404, "kein Bild"); }
  size_t len = keepLen;
  uint8_t* c = (uint8_t*)ps_malloc(len);
  if (!c) c = (uint8_t*)malloc(len);
  if (c) memcpy(c, keepJpg, len);
  UNLOCK();
  if (!c) return sendTxt(r, 500, "RAM");
  httpd_resp_set_type(r, "image/jpeg");
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  esp_err_t e = httpd_resp_send(r, (const char*)c, len);
  free(c);
  return e;
}
static esp_err_t hReboot(httpd_req_t* r) { sendTxt(r, 200, "OK"); Serial.println("[SYS] Neustart"); delay(300); ESP.restart(); return ESP_OK; }
static esp_err_t hReset(httpd_req_t* r) { LittleFS.remove("/config.json"); sendTxt(r, 200, "OK"); Serial.println("[SYS] Werkseinstellungen, Neustart"); delay(300); ESP.restart(); return ESP_OK; }
static esp_err_t hUpdate(httpd_req_t* r) {
  size_t len = r->content_len;
  if (len < 1000) return sendTxt(r, 400, "Datei fehlt");
  Serial.printf("[OTA] Start, %u Bytes\n", (unsigned)len);
  otaBusy = true;
  uint8_t* b = (uint8_t*)malloc(2048);
  if (!b) { otaBusy = false; return sendTxt(r, 500, "RAM"); }
  bool ok = Update.begin(len);
  size_t got = 0; int to = 0, lastp = -10;
  while (ok && got < len) {
    size_t want = (len - got) < 2048 ? (len - got) : 2048;
    int n = httpd_req_recv(r, (char*)b, want);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) { if (++to > 10) ok = false; continue; }
    to = 0;
    if (n <= 0 || Update.write(b, n) != (size_t)n) { ok = false; break; }
    got += n;
    int pc = (int)(got * 100 / len);
    if (pc >= lastp + 10) { lastp = pc; Serial.printf("[OTA] %d%%\n", pc); }
  }
  free(b);
  if (ok) ok = Update.end(true); else Update.abort();
  if (!ok) { Serial.printf("[OTA] FEHLER: %s\n", Update.errorString()); otaBusy = false; return sendTxt(r, 500, "FEHLER"); }
  sendTxt(r, 200, "Update OK");
  Serial.println("[OTA] fertig, Neustart");
  delay(500); ESP.restart();
  return ESP_OK;
}

void startHttp() {
  httpd_config_t c = HTTPD_DEFAULT_CONFIG();
  c.max_open_sockets = 5; c.lru_purge_enable = true; c.stack_size = 12288; c.max_uri_handlers = 16;
  if (httpd_start(&httpd, &c) != ESP_OK) { Serial.println("[HTTP] Start FEHLGESCHLAGEN"); return; }
  struct Route { const char* u; httpd_method_t m; esp_err_t (*h)(httpd_req_t*); };
  const Route routes[] = {
    {"/", HTTP_GET, hPage}, {"/api/config", HTTP_GET, hCfgGet}, {"/api/config", HTTP_POST, hCfgPost},
    {"/api/camdef", HTTP_GET, hCamDef}, {"/api/status", HTTP_GET, hStatus}, {"/snapshot.jpg", HTTP_GET, hSnap},
    {"/api/diag/info", HTTP_GET, hDiagInfo}, {"/api/diag/image", HTTP_GET, hDiagImage}, {"/api/reboot", HTTP_POST, hReboot}, {"/api/reset", HTTP_POST, hReset}, {"/update", HTTP_POST, hUpdate}
  };
  for (const Route& x : routes) {
    httpd_uri_t u; memset(&u, 0, sizeof(u));
    u.uri = x.u; u.method = x.m; u.handler = x.h;
    httpd_register_uri_handler(httpd, &u);
  }
  Serial.printf("[HTTP] Webinterface: http://%s/\n", (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str());
}

// ---------- WLAN-Diagnose (Serieller Monitor, 115200 Baud) ----------
#define WLAN_SCAN 0   // 1 = vor dem Verbinden alle Netze scannen und auflisten
const char* wifiReason(int r) {
  switch (r) {
    case 1: return "UNSPECIFIED";
    case 2: return "AUTH_EXPIRE (Authentifizierung abgelaufen)";
    case 3: return "AUTH_LEAVE";
    case 4: return "ASSOC_EXPIRE";
    case 5: return "ASSOC_TOOMANY (Router voll)";
    case 6: return "NOT_AUTHED";
    case 7: return "NOT_ASSOCED";
    case 8: return "ASSOC_LEAVE (Router hat getrennt)";
    case 14: return "MIC_FAILURE";
    case 15: return "4WAY_HANDSHAKE_TIMEOUT (meist: FALSCHES PASSWORT)";
    case 16: return "GROUP_KEY_UPDATE_TIMEOUT";
    case 17: return "IE_IN_4WAY_DIFFERS";
    case 23: return "802_1X_AUTH_FAILED";
    case 34: return "BEACON_TIMEOUT / schwaches Signal";
    case 39: return "TIMEOUT";
    case 200: return "BEACON_TIMEOUT (Router nicht mehr empfangen)";
    case 201: return "NO_AP_FOUND (SSID nicht gefunden: falsch geschrieben, 5 GHz-only, versteckt, zu weit weg)";
    case 202: return "AUTH_FAIL (Passwort falsch / Verschluesselung passt nicht)";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT (meist: FALSCHES PASSWORT)";
    case 205: return "CONNECTION_FAIL";
    case 210: return "NO_AP_FOUND_W_COMPATIBLE_SECURITY (Router nutzt WPA3-only o.ae.)";
    case 211: return "NO_AP_FOUND_IN_AUTHMODE_THRESHOLD";
    case 212: return "NO_AP_FOUND_IN_RSSI_THRESHOLD";
    default: return "(siehe esp_wifi_types.h)";
  }
}
const char* wifiStatusName(int s) {
  switch (s) {
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL (SSID nicht gefunden)";
    case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED (meist falsches Passwort)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "?";
  }
}
void onWiFiEvent(WiFiEvent_t ev, WiFiEventInfo_t info) {
  switch (ev) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WLAN] Mit Access Point verbunden (Passwort OK), warte auf IP per DHCP...");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WLAN] IP erhalten: %s\n", WiFi.localIP().toString().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[WLAN] Getrennt/Fehlgeschlagen, Grund %d: %s\n",
                    info.wifi_sta_disconnected.reason, wifiReason(info.wifi_sta_disconnected.reason));
      break;
    default: break;
  }
}
bool isHex64(const String& s) {
  if (s.length() != 64) return false;
  for (size_t i = 0; i < s.length(); i++) if (!isxdigit((unsigned char)s[i])) return false;
  return true;
}
void wifiConnect() {
  Serial.printf("\n[CFG] config.json vorhanden: %s\n", LittleFS.exists("/config.json") ? "ja" : "NEIN (Defaults)");
  Serial.printf("[WLAN] SSID='%s' (Laenge %d), Passwort-Laenge %d, Hostname '%s'\n",
                cfg.ssid.c_str(), cfg.ssid.length(), cfg.wpass.length(), cfg.host.c_str());
  if (cfg.ssid.isEmpty()) { Serial.println("[WLAN] Keine SSID konfiguriert -> Access-Point-Modus"); return; }

  int pl = cfg.wpass.length();
  if (pl > 0 && pl < 8) Serial.println("[WLAN] WARNUNG: WPA-Passwort muss mindestens 8 Zeichen haben");
  if (pl > 63 && !isHex64(cfg.wpass))
    Serial.println("[WLAN] WARNUNG: Passwort laenger als 63 Zeichen! WPA2 erlaubt max. 63 Zeichen (oder genau 64 Hex-Zeichen). Wird vom ESP32 NICHT akzeptiert.");
  String ts = cfg.ssid; ts.trim(); String tp = cfg.wpass; tp.trim();
  if (ts != cfg.ssid) Serial.println("[WLAN] WARNUNG: SSID hat Leerzeichen am Anfang/Ende");
  if (tp != cfg.wpass) Serial.println("[WLAN] WARNUNG: Passwort hat Leerzeichen am Anfang/Ende");
  for (size_t i = 0; i < cfg.wpass.length(); i++)
    if ((uint8_t)cfg.wpass[i] > 126) { Serial.println("[WLAN] WARNUNG: Passwort enthaelt Nicht-ASCII-Zeichen (Umlaute/Sonderzeichen)"); break; }

  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(cfg.host.c_str());
  WiFi.setSleep(false);

  if (WLAN_SCAN) {
  Serial.println("[WLAN] Scanne Netzwerke (nur 2,4 GHz sichtbar)...");
  int n = WiFi.scanNetworks();
  Serial.printf("[WLAN] Scan fertig, %d Netze\n", n);
  bool found = false;
  for (int i = 0; i < n; i++) {
    bool match = (WiFi.SSID(i) == cfg.ssid);
    Serial.printf("  %s '%s'  RSSI %d dBm  Kanal %d  Verschluesselung %d\n", match ? "->" : "  ",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i), (int)WiFi.encryptionType(i));
    if (match) found = true;
  }
  WiFi.scanDelete();
  if (!found) Serial.println("[WLAN] SSID im Scan NICHT gefunden (Schreibweise/Gross-Kleinschreibung? Nur 5 GHz? Router zu weit weg?)");
  else Serial.println("[WLAN] SSID gefunden. (Verschluesselung: 3=WPA2, 4=WPA/WPA2, 5=WPA2-Enterprise, 6=WPA3, 7=WPA2/WPA3)");
  }

  Serial.println("[WLAN] Verbinde...");
  WiFi.begin(cfg.ssid.c_str(), cfg.wpass.c_str());
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 30000) { delay(500); Serial.printf("  Status: %s\n", wifiStatusName(WiFi.status())); }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("[WLAN] VERBUNDEN. IP %s, RSSI %d dBm, Kanal %d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.channel());
  else
    Serial.printf("[WLAN] FEHLGESCHLAGEN, Status: %s\n", wifiStatusName(WiFi.status()));
}

void setup() {
  Serial.begin(115200);
  delay(800);
  mtx = xSemaphoreCreateMutex();
  Serial.printf("\n[SYS] ZaehlerCam v%s, Build %s %s\n", FW_VERSION, __DATE__, __TIME__);
  snprintf(rstTxt, sizeof(rstTxt), "%s", resetName(esp_reset_reason()));
  if (rtcMagic == 0xC0FFEE42) { size_t l = strlen(rstTxt); snprintf(rstTxt + l, sizeof(rstTxt) - l, ", Watchdog: %s", whyName(rtcWhy)); rtcMagic = 0; }
  Serial.printf("[SYS] Neustart-Grund: %s\n", rstTxt);
  cfgDefaults();
  LittleFS.begin(true);
  loadCfg();
  if (cfg.camEn) camInit();   // zuerst Kamera, danach LED-PWM (LEDC-Belegung)
  else Serial.println("[CAM] deaktiviert (Webinterface -> Kamera -> Kamera aktiv)");
  ledApply();

  wifiConnect();
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP_STA); WiFi.softAP("ZaehlerCam-Setup"); apMode = true;
    Serial.printf("[WLAN] Access Point 'ZaehlerCam-Setup' aktiv: http://%s\n", WiFi.softAPIP().toString().c_str());
  }
  MDNS.begin(cfg.host.c_str());
  mqReconf();

  startHttp();
  loopBeat = millis();
  xTaskCreate(wdTask, "wd", 4096, NULL, 1, NULL);
}

void loop() {
  loopBeat = millis();
  mqLoop();
  static uint32_t tw = 0, ts = 0, t = 0;
  if (millis() - tw > 15000) {
    tw = millis();
    if (cfg.ssid.length() && WiFi.status() != WL_CONNECTED)
      Serial.printf("[WLAN] nicht verbunden mit '%s', Status: %s\n", cfg.ssid.c_str(), wifiStatusName(WiFi.status()));
    else if (apMode && WiFi.status() == WL_CONNECTED) { apMode = false; WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); }
  }
  {   // WLAN ueberwachen: bei Verlust neu verbinden, nach 5 min ohne Verbindung neu starten
    static uint32_t downSince = 0, lastTry = 0;
    if (cfg.ssid.length() && WiFi.status() != WL_CONNECTED) {
      if (!downSince) downSince = millis();
      if (millis() - lastTry > 30000) {
        lastTry = millis(); wifiReconn++;
        Serial.printf("[WLAN] seit %lu s getrennt -> neuer Verbindungsversuch Nr. %u\n", (unsigned long)((millis() - downSince) / 1000), (unsigned)wifiReconn);
        WiFi.disconnect(false); WiFi.begin(cfg.ssid.c_str(), cfg.wpass.c_str());
      }
      if (!apMode && millis() - downSince > 300000UL) selfRestart(2, "WLAN seit 5 min nicht verbunden");
    } else downSince = 0;
  }
  {   // Diagnose per MQTT alle 5 min (und direkt nach dem Verbinden)
    static uint32_t td = 0; static bool first = true;
    if (mqUp && (first || millis() - td > 300000UL)) {
      first = false; td = millis();
      char b[300];
      snprintf(b, sizeof(b), "{\"fw\":\"%s\",\"uptime\":%lu,\"heap\":%u,\"minheap\":%u,\"psram\":%u,\"rssi\":%d,\"wifi_reconnects\":%u,\"reset\":\"%s\"}",
               FW_VERSION, (unsigned long)(millis() / 1000), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), (unsigned)ESP.getFreePsram(), WiFi.RSSI(), (unsigned)wifiReconn, rstTxt);
      mqPub("diag", String(b), false);
    }
    if (!mqUp) first = true;
  }
  if (millis() - ts > 30000) {
    ts = millis();
    Serial.printf("[SYS] Heap %u, PSRAM %u, RSSI %d dBm, Kamera %s, Display %s, Text '%s'\n", (unsigned)ESP.getFreeHeap(),
                  (unsigned)ESP.getFreePsram(), WiFi.RSSI(), camOk ? "OK" : (cfg.camEn ? "FEHLER" : "AUS"), res.on ? "AN" : "AUS", res.txt);
  }
  if (otaBusy) { delay(20); return; }
  if (millis() - t >= (uint32_t)cfg.dInt) {
    t = millis();
    LOCK();
    if (camOk) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) {
        analyze(fb); stepMachine(); lastAnTs = millis();
        if (res.on && (stt == ST_SAMPLE || stt == ST_DONE)) {
          // bei dauerhaft "an" alle 2 s auffrischen, sonst einmal pro Einschaltphase
          if (!capDone || millis() - keepT > 2000) { keepFrame(fb); capDone = true; }
        }
        else if (!res.on) capDone = false;
        esp_camera_fb_return(fb);
      }
    }
    UNLOCK();
  }
  delay(2);
}

// ======================= Weboberfläche =======================
const char PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html><html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>ZählerCam</title>
<style>
:root{color-scheme:light dark}
body{font-family:system-ui,sans-serif;margin:auto;padding:0 12px 40px;max-width:1000px}
header{position:sticky;top:0;background:Canvas;padding:8px 0;z-index:5;border-bottom:1px solid #8884}
nav{display:flex;gap:6px;flex-wrap:wrap}
nav button.act{outline:2px solid #28a;font-weight:bold}
button,input,select{font:inherit}
#stat{font-size:.85em;margin-top:6px}
fieldset{border:1px solid #8886;border-radius:8px;margin:10px 0;min-width:0;max-width:100%;box-sizing:border-box}
#smpList{max-width:100%;overflow-x:auto}#smpList table{font-size:.85em;border-collapse:collapse}#smpList td,#smpList th{padding:2px 6px;text-align:left;white-space:nowrap}
label{display:block;margin:5px 0}label>span{display:inline-block;min-width:170px}
input[type=range]{width:200px;vertical-align:middle}
input[type=text],input[type=password],input[type=number],select{padding:3px}
.pg{display:none}.pg.act{display:block}
#wrap{position:relative;display:block;width:100%}
#wrap img{display:block;width:100%;height:auto;background:#000}
#wrap canvas{position:absolute;left:0;top:0;width:100%;height:100%;cursor:crosshair;touch-action:none}
#im1{width:100%;height:auto;background:#000}
table{border-collapse:collapse}td,th{padding:2px 8px;text-align:center}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:0 16px}
</style></head><body>
<header><nav>
<button onclick="show(0)">1 · System</button><button onclick="show(1)">2 · Kamera</button><button onclick="show(2)">3 · Display &amp; Filter</button>
<button onclick="show(3)">4 · Diagnose</button>
<select id=langSel onchange="setLang(this.value)" style="margin-left:auto" title="Sprache / Language"><option value=de>Deutsch</option><option value=en>English</option></select>
<button onclick="push(true)">💾 Speichern</button></nav>
<div id="stat">…</div><div id="msg" style="font-size:.85em"></div></header>

<div class="pg act" id="p0">
<fieldset><legend>WLAN (Änderung gilt nach Neustart)</legend>
<label><span>SSID</span><input type=text autocomplete=off data-k="wifi.ssid"></label>
<label><span>Passwort</span><input type=password autocomplete=new-password data-k="wifi.pass"></label>
<label><span>Hostname</span><input type=text data-k="wifi.host"></label></fieldset>

<fieldset><legend>MQTT</legend>
<label><span>MQTT aktiv</span><input type=checkbox data-k="mqtt.en"></label>
<label><span>Broker</span><input type=text data-k="mqtt.host"></label>
<label><span>Port</span><input type=number data-k="mqtt.port"></label>
<label><span>Benutzer</span><input type=text data-k="mqtt.user"></label>
<label><span>Passwort</span><input type=password data-k="mqtt.pass"></label>
<label><span>Basis-Topic</span><input type=text data-k="mqtt.base"></label>
<label><span>Client-ID</span><input type=text data-k="mqtt.cid"></label>
<label><span>Retain (value/raw)</span><input type=checkbox data-k="mqtt.retain"></label>
<label><span>Immer senden</span><input type=checkbox data-k="mqtt.always"> <small>aus = nur bei geändertem Wert</small></label>
<small>Topics: &lt;base&gt;/value, /raw, /json, /display (ON/OFF), /seen (nur bei Fehllesung), /rejected (verworfen), /status (online/offline)</small></fieldset>
<fieldset><legend>Plausibilitätsprüfung</legend>
<label><span>Zähler darf nur hochzählen</span><input type=checkbox data-k="plaus.en"></label>
<label><span>Max. Anstieg je Wert (0 = unbegrenzt)</span><input type=number min=0 data-k="plaus.max"></label>
<label><span>Bestätigungen für neue Basis</span><input type=number min=1 max=20 data-k="plaus.conf"></label>
<label><span>Fehlende Ziffern aus letztem Wert ergänzen</span><input type=checkbox data-k="plaus.fill"></label>
<label><span>Unvollständige Werte (mit ?) senden</span><input type=checkbox data-k="plaus.partial"></label>
<small>Kleinere Werte oder zu große Sprünge werden verworfen (Topic /rejected), bis derselbe Wert so oft in Folge gelesen wurde wie eingestellt – dann gilt er als neue Basis (z. B. nach Zählertausch). Unvollständige Werte gehen nie auf /value, sondern nur auf /raw und /json (valid:false).</small></fieldset>

<fieldset><legend>LED-Ring</legend>
<label><span>LED an</span><input type=checkbox data-k="led.en"></label>
<label><span>Typ</span><select data-n data-k="led.type"><option value=1>WS2812B / NeoPixel (Datenleitung)</option><option value=0>Einfache LED(s) per PWM</option></select></label>
<label><span>GPIO-Pin (Daten)</span><input type=number data-k="led.pin"> <small>erlaubt: 2, 4, 12, 13, 14, 15, 33 · 12 = Strapping-Pin, Ring beim Booten nicht auf High ziehen</small></label>
<label><span>Helligkeit</span><input type=range min=0 max=100 data-k="led.pct"><b></b> %</label>
<label><span>Farbe (WS2812)</span><input type=color data-k="led.col"></label>
<label><span>Anzahl LEDs (WS2812)</span><input type=number min=1 max=64 data-k="led.count"></label>
<label><span>PWM-Frequenz (nur PWM)</span><input type=number data-k="led.freq"></label></fieldset>

<fieldset><legend>Firmware-Update</legend>
<input type=file id=fw accept=".bin"> <button onclick="ota()">Hochladen &amp; flashen</button>
<progress id=otap value=0 max=100></progress></fieldset>

<fieldset><legend>Einstellungen</legend>
<a href="/api/config?download=1" download>⬇ Export (JSON, enthält Passwörter!)</a><br><br>
Import: <input type=file accept=".json" onchange="imp(this.files[0])"><br><br>
<button onclick="if(confirm('Neustart?'))fetch('/api/reboot',{method:'POST'})">Neustart</button>
<button onclick="if(confirm('Alle Einstellungen löschen?'))fetch('/api/reset',{method:'POST'})">Werkseinstellungen</button></fieldset>
</div>

<div class="pg" id="p1"><div class="grid"><div><img id="im1"></div><div id="camctl"></div></div></div>

<div class="pg" id="p3">
<fieldset><legend>📦 Bilder-Paket (Momentaufnahme)</legend>
<p>Speichert Einstellungen (ohne Passwörter), aktuelle Messwerte, ein Livebild und das letzte Bild, bei dem die Anzeige als „an“ erkannt wurde – jeweils mit und ohne Einzeichnung.</p>
<button onclick="diag()">📦 Bilder-Paket speichern</button></fieldset>
<fieldset><legend>🎥 Aufnahme (Video / Bildfolge mit Messwerten)</legend>
<p>Nimmt die gewählte Zeit lang so schnell wie möglich Bilder auf und notiert zu jedem Bild die Messwerte der Firmware (Anzeige an/aus, Helligkeit, Kontrast, erkannte Ziffern, Zustand). Während der Aufnahme sollte die Anzeige mindestens einmal an- und wieder ausgehen.</p>
<label><span>Dauer (s)</span><input type=number id=recDur min=3 max=180 value=30 style="width:6em"></label>
<label><span>Mindestabstand zwischen Bildern (ms)</span><input type=number id=recGap min=0 max=2000 value=100 style="width:6em"></label>
<label><span>JPEG-Qualität</span><input type=number id=recQ min=10 max=95 value=75 style="width:6em"></label>
<label><span>Video (.webm, mit Einblendung)</span><input type=checkbox id=recVid checked></label>
<label><span>Einzelbilder + Messwerte (.zip)</span><input type=checkbox id=recZip checked></label>
<p><button id=recBtn onclick="recStart()" style="font-weight:bold">🎥 Aufnahme starten</button> <button id=recStop onclick="recAbort=true" disabled>⏹ Beenden</button>
<progress id=recProg value=0 max=100 style="width:200px"></progress> <span id=recInfo></span></p>
<img id=recImg style="max-width:100%;width:480px;background:#000" alt="">
<p><small>Die Datei(en) schicken – die Messwerte (<code>messwerte.csv</code> im ZIP) zeigen Bild für Bild, wann die Firmware „an“ erkannt hat. Bei 30 s und etwa 8 Bildern pro Sekunde entstehen rund 10 MB.</small></p></fieldset>
</div>

<div class="pg" id="p2">
<div id="wrap"><img id="im2"><canvas id="cv"></canvas></div>
<div id="holdBox" style="margin:8px 0">
<button id=holdBtn onclick="toggleHold()" style="font-weight:bold">⏸ Hold</button>
<label style="display:inline;margin-left:12px"><input type=checkbox data-k="disp.lock"> 🔒 Rahmen &amp; Raster sperren</label>
<span id=lockInfo style="margin-left:6px;font-size:.85em"></span><br>
<span id=holdInfo style="font-size:.9em"></span>
<div id=holdLbl style="display:none;margin-top:6px">Zustand dieses Bildes festlegen:
 <button onclick="addSample('off')">⚫ Anzeige ist AUS</button> <button onclick="addSample('on')">🟢 Anzeige ist AN</button></div>
</div>
<p><b>Bearbeiten:</b>
<label style="display:inline"><input type=radio name=mode value=disp checked> Display-Rahmen</label>
<label style="display:inline"><input type=radio name=mode value=dig> Ziffer verschieben</label>
<label style="display:inline"><input type=radio name=mode value=seg> Einzelnes Segment</label>
&nbsp; <label style="display:inline"><input type=checkbox id=bin> Filter-Vorschau (so sieht die Auswertung das Bild: Segmente schwarz)</label><br>
<small>Tipp: Die Anzeige blinkt nur kurz – „📦 Diagnose“ nutzt automatisch das zuletzt aufgenommene Bild, bei dem die Anzeige an war.<br>Rahmen mit der Maus aufziehen, verschieben, an den Ecken skalieren. Grün = Segment an, rot = aus.</small></p>
<div class="grid">
<fieldset><legend>Display</legend>
<label><span>Anzahl Ziffern</span><input type=number min=1 max=8 data-k="disp.n"></label>
<label><span>Nachkommastellen</span><input type=number min=0 max=7 data-k="disp.dec"></label>
<label><span>Drehung (°)</span><input type=range id=rotR min=-20 max=20 step=0.05 style="width:160px"> <button type=button onclick="setRot(cfg.disp.rot-0.05)">−</button><input type=number id=rotN step=0.05 style="width:5.5em"><button type=button onclick="setRot(cfg.disp.rot+0.05)">+</button></label>
<label><span>Ziffernabstand ‰</span><input type=number data-k="disp.lay.0"></label>
<label><span>Segmentbreite ‰</span><input type=number data-k="disp.lay.1"></label>
<label><span>Segmenthöhe ‰</span><input type=number data-k="disp.lay.2"></label>
<label><span>Rand X ‰</span><input type=number data-k="disp.lay.3"></label>
<label><span>Rand Y ‰</span><input type=number data-k="disp.lay.4"></label>
<label><span>Kursiv ‰ (z. B. 100–250)</span><input type=number data-k="disp.lay.5"></label>
<small>Änderungen an Abstand, Breite, Höhe, Rand und Kursiv erzeugen das Raster sofort neu (manuelle Verschiebungen gehen verloren).</small><br><button id=autoBtn onclick="auto()">Segment-Raster neu erzeugen</button> <button id=calBtn onclick="calib()" style="font-weight:bold">🎯 Auto-Kalibrierung</button><br><small>Zieh vorher den blauen Rahmen grob um die Ziffern (darf ruhig 30 % zu groß sein). Die Kalibrierung sammelt so lange Bilder, bis alle Ziffern sicher gelesen werden – höchstens <input type=number id=calDur min=5 max=120 value=30 style="width:4.5em"> s (Anzeige dafür dauerhaft an lassen, wenn möglich).</small></fieldset>
<fieldset><legend>Filter / Auswertung</legend>
<label><span>Verfahren</span><select data-n data-k="flt.mode"><option value=2>Hintergrund-Ausgleich (adaptiv, empfohlen)</option><option value=1>Lokaler Kontrast</option><option value=0>Globale Schwelle</option></select></label>
<label><span>Ausgleich-Radius (px, Modus Hintergrund)</span><input type=number min=3 max=60 data-k="flt.rad"></label>
<label><span>Kontrast Δ (nur Modus lokaler Kontrast)</span><input type=range min=1 max=60 data-k="flt.delta"><b></b> Graustufen</label>
<label><span>Schwelle automatisch</span><input type=checkbox data-k="flt.auto"></label>
<label><span>Schwelle manuell (Hintergrund: 1–60, global: 0–255)</span><input type=range min=0 max=255 data-k="flt.thr"><b></b></label>
<label><span>Invertieren</span><input type=checkbox data-k="flt.inv"> <small>an = dunkle Segmente auf hellem Grund</small></label>
<label><span>Segment „an“ ab (Hintergrund/global)</span><input type=range min=1 max=100 data-k="flt.pct"><b></b> % Pixel</label>
<label><span>Fehlertoleranz (Segmente)</span><input type=number min=0 max=3 data-k="flt.tol"></label></fieldset>
<fieldset><legend>Display-an-Erkennung</legend>
<label><span>Methode</span><select data-n data-k="det.mode"><option value=0>Helligkeit ≥ Wert</option><option value=1>Helligkeit ≤ Wert</option><option value=2>Kontrast ≥ Wert</option><option value=5>Ziffern erkannt (Mindestanzahl gültiger Ziffern)</option><option value=3>Helligkeits-Sprung gegenüber Ruhewert (adaptiv)</option><option value=4>Kontrast-Sprung gegenüber Ruhewert (adaptiv)</option></select></label>
<label><span>Wert (Ziffern-Anzahl / Sprung / Schwelle)</span><input type=range min=0 max=255 data-k="det.lvl"><b></b></label>
<label><span>Einschwingzeit (ms)</span><input type=number data-k="det.settle"></label>
<label><span>Messungen (Mehrheit)</span><input type=number min=1 max=16 data-k="det.samples"></label>
<label><span>Bildintervall (ms)</span><input type=number data-k="det.interval"></label>
<label><span>Neu lesen alle (s, 0=aus)</span><input type=number data-k="det.reeval"></label></fieldset>
<fieldset><legend>Display-an-Erkennung: Muster &amp; Kalibrierung</legend>
<small>Mit <b>⏸ Hold</b> ein Bild einfrieren und als AN oder AUS markieren. Von jedem Zustand mindestens ein Bild (besser drei und mehr, auch bei anderem Licht). Dann kalibrieren.</small>
<div id=smpList style="margin:6px 0"></div>
<button id=smpCal onclick="calDetect()" disabled>🎯 Erkennung „Display an“ kalibrieren</button> <button onclick="smpUndo()">↩ Letztes Muster löschen</button> <button onclick="smpClear()">🗑 Alle Muster zurücksetzen</button>
<div id=smpRes style="margin-top:6px;font-size:.9em"></div></fieldset>
<fieldset><legend>Ziffern-Muster (Segmente a–g)</legend><div id="pat"></div>
<small>a=oben, b=oben rechts, c=unten rechts, d=unten, e=unten links, f=oben links, g=Mitte</small></fieldset>
</div></div>

<script>
let cfg,camdef,st=null,tab=0,busy=false,sel=null,drag=null,T;
const $=id=>document.getElementById(id);
const get=(o,p)=>p.split('.').reduce((a,k)=>a[k],o);
const set=(o,p,v)=>{const k=p.split('.'),l=k.pop();k.reduce((a,x)=>a[x],o)[l]=v};

// ---------- Sprache / Language (DE/EN) ----------
const I18N_EXACT=new Map([['% Pixel','% pixels'],['1 · System','1 · System'],['2 · Kamera','2 · Camera'],['3 · Display & Filter','3 · Display & Filter'],['4 · Diagnose','4 · Diagnostics'],
['Anzahl LEDs (WS2812)','Number of LEDs (WS2812)'],['Anzahl Ziffern','Number of digits'],['Ausgleich-Radius (px, Modus Hintergrund)','Compensation radius (px, background mode)'],
['Basis-Topic','Base topic'],['Bearbeiten:','Edit:'],['Benutzer','User'],['Bildintervall (ms)','Frame interval (ms)'],['Dauer (s)','Duration (s)'],
['Die Datei(en) schicken – die Messwerte (','Send the file(s) – the measurements ('],
['im ZIP) zeigen Bild für Bild, wann die Firmware „an“ erkannt hat. Bei 30 s und etwa 8 Bildern pro Sekunde entstehen rund 10 MB.','in the ZIP) show frame by frame when the firmware detected “on”. 30 s at about 8 frames per second produce roughly 10 MB.'],
['Display-Rahmen','Display frame'],['Display-an-Erkennung: Muster & Kalibrierung','Display-on detection: samples & calibration'],['Display-an-Erkennung','Display-on detection'],
['Drehung (°)','Rotation (°)'],['Einfache LED(s) per PWM','Simple LED(s) via PWM'],['Einschwingzeit (ms)','Settling time (ms)'],['Einstellungen','Settings'],
['Einzelbilder + Messwerte (.zip)','Single images + measurements (.zip)'],['Einzelnes Segment','Single segment'],['Farbe (WS2812)','Colour (WS2812)'],
['Fehlertoleranz (Segmente)','Error tolerance (segments)'],['Filter / Auswertung','Filter / evaluation'],
['Filter-Vorschau (so sieht die Auswertung das Bild: Segmente schwarz)','Filter preview (what the evaluation sees: segments black)'],
['Firmware-Update','Firmware update'],['GPIO-Pin (Daten)','GPIO pin (data)'],['Globale Schwelle','Global threshold'],['Graustufen','Grayscale'],
['Helligkeit','Brightness'],['Helligkeit ≤ Wert','Brightness ≤ value'],['Helligkeit ≥ Wert','Brightness ≥ value'],['Helligkeits-Sprung gegenüber Ruhewert (adaptiv)','Brightness jump vs. baseline (adaptive)'],
['Hintergrund-Ausgleich (adaptiv, empfohlen)','Background compensation (adaptive, recommended)'],['Hochladen & flashen','Upload & flash'],
['Immer senden','Always send'],['Import:','Import:'],['Invertieren','Invert'],['JPEG-Qualität','JPEG quality'],['Kontrast Δ (nur Modus lokaler Kontrast)','Contrast Δ (local contrast mode only)'],
['Kontrast ≥ Wert','Contrast ≥ value'],['Kontrast-Sprung gegenüber Ruhewert (adaptiv)','Contrast jump vs. baseline (adaptive)'],['Kursiv ‰ (z. B. 100–250)','Slant ‰ (e.g. 100–250)'],
['LED an','LED on'],['Lokaler Kontrast','Local contrast'],['MQTT aktiv','MQTT enabled'],['Messungen (Mehrheit)','Measurements (majority)'],['Methode','Method'],
['Mindestabstand zwischen Bildern (ms)','Minimum gap between frames (ms)'],['Mit','Use'],['Nachkommastellen','Decimal places'],['Neu lesen alle (s, 0=aus)','Re-read every (s, 0 = off)'],['Neustart','Restart'],
['Nimmt die gewählte Zeit lang so schnell wie möglich Bilder auf und notiert zu jedem Bild die Messwerte der Firmware (Anzeige an/aus, Helligkeit, Kontrast, erkannte Ziffern, Zustand). Während der Aufnahme sollte die Anzeige mindestens einmal an- und wieder ausgehen.','Records frames as fast as possible for the chosen time and notes the firmware measurements for every frame (display on/off, brightness, contrast, recognised digits, state). During the recording the display should switch on and off at least once.'],
['PWM-Frequenz (nur PWM)','PWM frequency (PWM only)'],['Passwort','Password'],['Rahmen mit der Maus aufziehen, verschieben, an den Ecken skalieren. Grün = Segment an, rot = aus.','Drag the frame with the mouse, move it, scale it at the corners. Green = segment on, red = off.'],
['Rand X ‰','Margin X ‰'],['Rand Y ‰','Margin Y ‰'],['Retain (value/raw)','Retain (value/raw)'],['Schwelle automatisch','Automatic threshold'],
['Schwelle manuell (Hintergrund: 1–60, global: 0–255)','Manual threshold (background: 1–60, global: 0–255)'],['Segment „an“ ab (Hintergrund/global)','Segment “on” from (background/global)'],
['Segment-Raster neu erzeugen','Regenerate segment grid'],['Segmentbreite ‰','Segment width ‰'],['Segmenthöhe ‰','Segment height ‰'],
['Speichert Einstellungen (ohne Passwörter), aktuelle Messwerte, ein Livebild und das letzte Bild, bei dem die Anzeige als „an“ erkannt wurde – jeweils mit und ohne Einzeichnung.','Saves settings (without passwords), current measurements, a live image and the last image in which the display was detected as “on” – each with and without overlay.'],
['Tipp: Die Anzeige blinkt nur kurz – „📦 Diagnose“ nutzt automatisch das zuletzt aufgenommene Bild, bei dem die Anzeige an war.','Tip: the display is only visible briefly – “📦 Diagnostics” automatically uses the last captured image in which the display was on.'],
['Topics: <base>/value, /raw, /json, /display (ON/OFF), /seen (nur bei Fehllesung), /rejected (verworfen), /status (online/offline)','Topics: <base>/value, /raw, /json, /display (ON/OFF), /seen (only on misread), /rejected (discarded), /status (online/offline)'],['Plausibilitätsprüfung','Plausibility check'],['Zähler darf nur hochzählen','Counter may only count up'],['Max. Anstieg je Wert (0 = unbegrenzt)','Max. increase per reading (0 = unlimited)'],['Bestätigungen für neue Basis','Confirmations for a new baseline'],['Fehlende Ziffern aus letztem Wert ergänzen','Fill missing digits from the last value'],['Unvollständige Werte (mit ?) senden','Send incomplete values (with ?)'],['Kleinere Werte oder zu große Sprünge werden verworfen (Topic /rejected), bis derselbe Wert so oft in Folge gelesen wurde wie eingestellt – dann gilt er als neue Basis (z. B. nach Zählertausch). Unvollständige Werte gehen nie auf /value, sondern nur auf /raw und /json (valid:false).','Smaller values or too large jumps are discarded (topic /rejected) until the same value has been read as many times in a row as configured – then it becomes the new baseline (e.g. after replacing the meter). Incomplete values never go to /value, only to /raw and /json (valid:false).'],
['Typ','Type'],['Verfahren','Method'],['Video (.webm, mit Einblendung)','Video (.webm, with overlay)'],['WLAN (Änderung gilt nach Neustart)','WLAN (changes apply after restart)'],
['WS2812B / NeoPixel (Datenleitung)','WS2812B / NeoPixel (data line)'],['Werkseinstellungen','Factory reset'],['Wert (Ziffern-Anzahl / Sprung / Schwelle)','Value (digit count / jump / threshold)'],
['Zieh vorher den blauen Rahmen grob um die Ziffern (darf ruhig 30 % zu groß sein). Die Kalibrierung sammelt so lange Bilder, bis alle Ziffern sicher gelesen werden – höchstens','First drag the blue frame roughly around the digits (it may be 30 % too large). The calibration collects images until all digits are read reliably – at most'],
['s (Anzeige dafür dauerhaft an lassen, wenn möglich).','s (keep the display permanently on if possible).'],
['Ziffer verschieben','Move digit'],['Ziffern erkannt (Mindestanzahl gültiger Ziffern)','Digits recognised (minimum number of valid digits)'],['Ziffern-Muster (Segmente a–g)','Digit patterns (segments a–g)'],
['Ziffernabstand ‰','Digit gap ‰'],['Zustand dieses Bildes festlegen:','Set the state of this image:'],['ZählerCam','MeterCam'],
['a=oben, b=oben rechts, c=unten rechts, d=unten, e=unten links, f=oben links, g=Mitte','a=top, b=top right, c=bottom right, d=bottom, e=bottom left, f=top left, g=middle'],
['an = dunkle Segmente auf hellem Grund','on = dark segments on a light background'],['aus = nur bei geändertem Wert','off = only when the value changed'],
['ein Bild einfrieren und als AN oder AUS markieren. Von jedem Zustand mindestens ein Bild (besser drei und mehr, auch bei anderem Licht). Dann kalibrieren.','to freeze an image and mark it as ON or OFF. Take at least one image per state (better three or more, also under different lighting). Then calibrate.'],
['erlaubt: 2, 4, 12, 13, 14, 15, 33 · 12 = Strapping-Pin, Ring beim Booten nicht auf High ziehen','allowed: 2, 4, 12, 13, 14, 15, 33 · 12 = strapping pin, do not pull the ring high during boot'],
['messwerte.csv','messwerte.csv'],['Änderungen an Abstand, Breite, Höhe, Rand und Kursiv erzeugen das Raster sofort neu (manuelle Verschiebungen gehen verloren).','Changes to gap, width, height, margin and slant regenerate the grid immediately (manual shifts are lost).'],
['↩ Letztes Muster löschen','↩ Delete last sample'],['⏸ Hold','⏸ Hold'],['⏹ Beenden','⏹ Stop'],['⚫ Anzeige ist AUS','⚫ Display is OFF'],['⬇ Export (JSON, enthält Passwörter!)','⬇ Export (JSON, contains passwords!)'],
['🎥 Aufnahme (Video / Bildfolge mit Messwerten)','🎥 Recording (video / image sequence with measurements)'],['🎥 Aufnahme starten','🎥 Start recording'],['🎯 Auto-Kalibrierung','🎯 Auto-calibration'],
['🎯 Erkennung „Display an“ kalibrieren','🎯 Calibrate “display on” detection'],['💾 Speichern','💾 Save'],['📦 Bilder-Paket (Momentaufnahme)','📦 Image package (snapshot)'],['📦 Bilder-Paket speichern','📦 Save image package'],
['🔒 Rahmen & Raster sperren','🔒 Lock frame & grid'],['🗑 Alle Muster zurücksetzen','🗑 Reset all samples'],['🟢 Anzeige ist AN','🟢 Display is ON'],
['Kamera aktiv','Camera enabled'],['Auflösung','Resolution'],['aus = keine Aufnahme, keine Auswertung','off = no capture, no evaluation'],
['AN','ON'],['AUS','OFF'],['🟢 AN','🟢 ON'],['⚫ AUS','⚫ OFF'],['Zustand','State'],['Kontrast','Contrast'],['Ziffern','Digits'],['Segm.','Seg.'],['Text','Text'],
['Ergebnis','Result'],['Übernommen:','Applied:'],['⚠ Keine saubere Trennung gefunden.','⚠ No clean separation found.'],['nicht','not'],['Noch keine Muster.','No samples yet.'],
['▶ Weiter','▶ Resume'],['Neustart?','Restart?'],['Alle Einstellungen löschen?','Delete all settings?'],
['Prüfe: Sind die Bilder richtig als AN/AUS markiert? Sind in den AN-Bildern alle Ziffern lesbar (sonst erst 🎯 Auto-Kalibrierung der Ziffern)? Gleiches Licht in beiden Zuständen? Weitere Muster aufnehmen. Die Einstellungen wurden','Check: are the images marked correctly as ON/OFF? Are all digits readable in the ON images (otherwise run 🎯 auto-calibration of the digits first)? Same lighting in both states? Record more samples. The settings were'],
['geändert.','changed.'],
['gesperrt: Rahmen, Drehung und Raster sind geschützt','locked: frame, rotation and grid are protected']]);
const I18N_PH=[['Änderung wird gesendet…','Sending change…'],['Live übernommen ✓ (noch nicht gespeichert) ','Applied live ✓ (not saved yet) '],['Gespeichert ✓ ','Saved ✓ '],['Importiert & gespeichert ✓','Imported & saved ✓'],
['Ungültige Datei','Invalid file'],['Verbindungsfehler','Connection error'],['Fehler: ','Error: '],['Fehler ','Error '],[' – Neustart…',' – restarting…'],
['Kamera ist aus – bitte erst auf Seite 2 einschalten','Camera is off – please switch it on first on page 2'],['Kamera ist aus – bitte erst einschalten','Camera is off – please switch it on first'],
['🔒 Gesperrt – erst den Haken „Rahmen & Raster sperren“ lösen, dann kalibrieren','🔒 Locked – first untick “Lock frame & grid”, then calibrate'],
['🔒 Gesperrt – erst den Haken „Rahmen & Raster sperren“ lösen','🔒 Locked – first untick “Lock frame & grid”'],
['🔒 Rahmen gesperrt – Haken „Rahmen & Raster sperren“ lösen, um zu ändern','🔒 Frame locked – untick “Lock frame & grid” to change it'],
['Kein gehaltenes Bild mit Messwerten – erst ⏸ Hold drücken','No held image with measurements – press ⏸ Hold first'],
['Je mindestens ein AN- und ein AUS-Muster nötig','At least one ON and one OFF sample is required'],
['Kalibrierung: zweiter Versuch mit größerem Suchbereich…','Calibration: second attempt with a larger search area…'],
['Kalibrierung fehlgeschlagen: kein Bild','Calibration failed: no image'],['Kalibrierung fehlgeschlagen: ','Calibration failed: '],
['Kalibrierung: Bild ','Calibration: image '],[' wird ausgewertet… (bestes bisher: ',' is being evaluated… (best so far: '],['Kalibriert: ','Calibrated: '],[', Drehung ',', rotation '],[', Reserve ',', margin '],['Kontrast-Δ','contrast Δ'],
[' ✓ (noch nicht gespeichert – 💾 Speichern)',' ✓ (not saved yet – 💾 Save)'],
[' – ⚠ nicht alle Ziffern gelesen: Rahmen näher um die Ziffern ziehen, Schärfe/Licht prüfen, dann nochmal',' – ⚠ not all digits read: draw the frame closer around the digits, check focus/lighting, then try again'],
['Diagnose wird erstellt…','Creating diagnostics…'],['Diagnose gespeichert: ','Diagnostics saved: '],[' Dateien ✓',' files ✓'],[' (kein Bild – Kamera aus?)',' (no image – camera off?)'],['Diagnose fehlgeschlagen: ','Diagnostics failed: '],
['Bitte Video und/oder ZIP wählen','Please choose video and/or ZIP'],['Video nicht unterstützt – nur ZIP','Video not supported – ZIP only'],['Aufnahme gespeichert ✓','Recording saved ✓'],['Aufnahme fehlgeschlagen: ','Recording failed: '],
['Fertig: ','Done: '],[' Bilder (',' images ('],[' Datei(en) gespeichert',' file(s) saved'],[' Bilder, ',' images, '],[' Bilder/s, ',' images/s, '],['Wechsel an↔aus','on↔off changes'],
['Gehaltenes Bild ohne Messwerte (noch kein Bild geladen oder Firmware zu alt)','Held image without measurements (no image loaded yet or firmware too old)'],
['Gehaltenes Bild: ','Held image: '],[', Ziffern ',', digits '],[', erkannt ',', detected '],[' – die Erkennung sagt: ',' – the detection says: '],[' – die Erkennung sagt: AUS',' – the detection says: OFF'],[' – die Erkennung sagt: AN',' – the detection says: ON'],
['Helligkeit (dunkler bei AN)','Brightness (darker when ON)'],['Ziffern erkannt: ','Digits recognised: '],[' → nicht trennbar',' → not separable'],[' → trennbar',' → separable'],[' → zu knapp',' → too close'],[' → überlappen',' → overlap'],[' → nein',' → no'],
['AN ≥ ','ON ≥ '],['AUS ≤ ','OFF ≤ '],['AN ≤ ','ON ≤ '],['AUS ≥ ','OFF ≥ '],[' von ',' of '],
['Verfahren „Ziffern erkannt“, mindestens ','Method “digits recognised”, at least '],['Verfahren „Helligkeit ≤ Wert“, Wert ','Method “brightness ≤ value”, value '],['Verfahren „','Method “'],[' ≥ Wert“, Wert ',' ≥ value”, value '],
[' (Trennreserve ',' (separation margin '],['Noch nicht gespeichert – 💾 Speichern.','Not saved yet – 💾 Save.'],
['Hinweis: Geringe Reserve – die Erkennung kann im Betrieb schwanken.','Note: small margin – the detection may fluctuate in operation.'],
['Hinweis: In den AN-Bildern werden nur ','Note: in the ON images only '],[' Ziffern gelesen. Wenn die Anzeige da komplett sichtbar war, erst die Ziffern kalibrieren (🎯 Auto-Kalibrierung).',' digits are read. If the display was fully visible there, calibrate the digits first (🎯 Auto-calibration).'],
['Hinweis: Nur ','Note: only '],[' AN- und ',' ON and '],[' AUS-Muster – für sichere Werte je 3 oder mehr aufnehmen, auch bei anderem Licht.',' OFF samples – for reliable values record 3 or more of each, also under different lighting.'],
['Laufzeit','Uptime'],['WLAN-Neuverbindungen','WLAN reconnects'],['Neustart-Grund','Reset reason'],['Software-Neustart','software restart'],['ABSTURZ (Panic/Exception)','CRASH (panic/exception)'],['BROWNOUT (Spannungseinbruch!)','BROWNOUT (voltage dip!)'],['Reset-Taste','reset button'],['Einschalten','power-on'],['Interrupt-Watchdog','interrupt watchdog'],['Task-Watchdog','task watchdog'],['Hauptschleife haengt','main loop hung'],['WLAN dauerhaft weg','WLAN lost permanently'],['zu wenig Heap','low heap'],['Helligkeit','Brightness'],['Kontrast','Contrast'],['Schwelle','Threshold'],['Ruhewert','Baseline'],['Erkannt','Detected'],['Letzter Wert','Last value'],['verbunden','connected'],['KAMERA FEHLER','CAMERA ERROR'],['Kamera aus','Camera off']].sort((a,b)=>b[0].length-a[0].length);
const I18N_RX=[[/^Muster gespeichert: (AN|AUS) \(jetzt (\d+) an \/ (\d+) aus\)$/,(m,a,b,c)=>'Sample saved: '+(a=='AN'?'ON':'OFF')+' (now '+b+' on / '+c+' off)']];
let LANG='de';const _lo=new WeakMap();
function tr(s){if(LANG!='en'||!s)return s;const m=s.match(/^(\s*)([\s\S]*?)(\s*)$/),c=m[2];if(!c)return s;
 if(I18N_EXACT.has(c))return m[1]+I18N_EXACT.get(c)+m[3];
 for(const [r,f] of I18N_RX){const x=c.match(r);if(x)return m[1]+f(...x)+m[3]}
 let o=c;for(const [de,en] of I18N_PH)if(o.includes(de))o=o.split(de).join(en);return m[1]+o+m[3]}
function trNode(n){const v=n.nodeValue,o=_lo.get(n);
 if(o&&v===o.out){const w=LANG=='en'?tr(o.src):o.src;if(w!==v){n.nodeValue=w;_lo.set(n,{src:o.src,out:w})}return}
 const w=LANG=='en'?tr(v):v;if(w!==v){_lo.set(n,{src:v,out:w});n.nodeValue=w}else if(o)_lo.delete(n)}
function trTree(r){if(r.nodeType==3){trNode(r);return}if(r.nodeType!=1||/^(SCRIPT|STYLE)$/.test(r.tagName))return;
 const w=document.createTreeWalker(r,NodeFilter.SHOW_TEXT);const l=[];let n;while(n=w.nextNode())l.push(n);l.forEach(trNode)}
new MutationObserver(ms=>{for(const m of ms){if(m.type=='characterData')trNode(m.target);else m.addedNodes.forEach(trTree)}}).observe(document.body,{childList:true,subtree:true,characterData:true});
const _cf=window.confirm.bind(window);window.confirm=m=>_cf(tr(m));
function setLang(v,quiet){LANG=v=='en'?'en':'de';try{localStorage.setItem('zc_lang',LANG)}catch(e){}
 document.documentElement.lang=LANG;const sel=$('langSel');if(sel)sel.value=LANG;if(cfg){cfg.lang=LANG}trTree(document.body)}
function initLang(){let v=cfg&&cfg.lang;if(!v){try{v=localStorage.getItem('zc_lang')}catch(e){}}
 if(!v)v=(navigator.language||'en').toLowerCase().startsWith('de')?'de':'en';setLang(v,true)}
const msg=t=>$('msg').textContent=t;
const cl=v=>Math.round(Math.min(1000,Math.max(0,v)));
function push(save){clearTimeout(T);msg('Änderung wird gesendet…');T=setTimeout(async()=>{try{const r=await fetch('/api/config?save='+(save?1:0),{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(cfg)});msg((r.ok?(save?'Gespeichert ✓ ':'Live übernommen ✓ (noch nicht gespeichert) '):'Fehler '+r.status+' ')+new Date().toLocaleTimeString())}catch(e){msg('Verbindungsfehler')}},save?0:200)}
function bind(){document.querySelectorAll('[data-k]').forEach(e=>{
 const v=get(cfg,e.dataset.k);if(e.type=='checkbox')e.checked=!!v;else e.value=v;
 const o=e.nextElementSibling,ob=o&&o.tagName=='B'?o:null;if(ob)ob.textContent=e.value;
 const h=()=>{const x=e.type=='checkbox'?e.checked:(e.type=='number'||e.type=='range'||e.dataset.n!==undefined)?Number(e.value):e.value;
  set(cfg,e.dataset.k,x);if(ob)ob.textContent=e.value;if((e.dataset.k=='disp.n'&&cfg.disp.seg.length!=x)||e.dataset.k.startsWith('disp.lay.'))auto();else push(false);if(e.dataset.k=='cam.en')setTimeout(frame,1500);if(e.dataset.k=='disp.lock')applyLockUI();draw()};
 e.onchange=h;if(e.type=='range')e.oninput=h})}
async function load(){cfg=await (await fetch('/api/config')).json();initLang();camdef=await (await fetch('/api/camdef')).json();
 let h='<label><span>Kamera aktiv</span><input type=checkbox data-k="cam.en"> <small>aus = keine Aufnahme, keine Auswertung</small></label><label><span>Auflösung</span><select data-k="cam.fs">'+['QVGA','VGA','SVGA','XGA','SXGA'].map(s=>`<option>${s}</option>`).join('')+'</select></label>';
 camdef.forEach(d=>{h+=`<label><span>${d.n}</span><input type=range min=${d.mn} max=${d.mx} data-k="cam.${d.n}"><b></b></label>`});
 if(cfg.disp.rot===undefined)cfg.disp.rot=0;if(cfg.disp.lock===undefined)cfg.disp.lock=false;$('rotR').value=cfg.disp.rot;$('rotN').value=cfg.disp.rot;$('camctl').innerHTML=h;buildPat();bind();applyLockUI();if(cfg.disp.seg.length!=cfg.disp.n&&!cfg.disp.lock)auto();draw()}
function buildPat(){let h='<table><tr><th></th>'+'abcdefg'.split('').map(c=>`<th>${c}</th>`).join('')+'</tr>';
 for(let k=0;k<10;k++){h+=`<tr><th>${k}</th>`;for(let s=0;s<7;s++)h+=`<td><input type=checkbox data-p=${k} data-s=${s} ${(cfg.disp.pat[k]>>s)&1?'checked':''}></td>`;h+='</tr>'}
 $('pat').innerHTML=h+'</table>';$('pat').onchange=e=>{const t=e.target;cfg.disp.pat[+t.dataset.p]^=(1<<+t.dataset.s);push(false)}}
function auto(){if(cfg.disp.lock){msg('🔒 Gesperrt – erst den Haken „Rahmen & Raster sperren“ lösen');return}const D=cfg.disp,L=D.lay,N=D.n,g=L[0],tx=L[1]/1000,ty=L[2]/1000,mx=L[3],my=L[4],sk=L[5]/1000,cw=(1000-(N-1)*g)/N;D.seg=[];
 for(let d=0;d<N;d++){const bx=d*(cw+g)+mx,bw=cw-2*mx,bh=1000-2*my;
  const U=[[tx,0,1-2*tx,ty],[1-tx,ty,tx,.5-1.5*ty],[1-tx,.5+ty/2,tx,.5-1.5*ty],[tx,1-ty,1-2*tx,ty],[0,.5+ty/2,tx,.5-1.5*ty],[0,ty,tx,.5-1.5*ty],[tx,.5-ty/2,1-2*tx,ty]];
  D.seg.push(U.map(u=>{const sh=sk*(1-u[1]-u[3]/2);return[Math.round(bx+(u[0]+sh)*bw),Math.round(my+u[1]*bh),Math.max(1,Math.round(u[2]*bw)),Math.max(1,Math.round(u[3]*bh))]}))}
 draw();push(false)}
function show(i){tab=i;if(hold&&i!=2)toggleHold();[0,1,2,3].forEach(j=>$('p'+j).classList.toggle('act',j==i));
 document.querySelectorAll('nav button').forEach((b,j)=>b.classList.toggle('act',j==i));if(i==1||i==2)frame()}
let tb=0,hold=false,heldMeta=null,heldBlob=null,lastMeta=null,lastBlob=null,samples=[];
function frame(){if((tab!=1&&tab!=2)||busy||hold||!cfg||!cfg.cam.en)return;busy=true;tb=Date.now();
 if(tab==2){frame2();return}
 $('im1').src='/snapshot.jpg?'+Date.now()}
async function frame2(){
 try{const r=await fetch('/snapshot.jpg?m=1'+($('bin').checked?'&bin=1':'')+'&r='+Date.now());if(!r.ok)throw new Error('HTTP '+r.status);
  let m=null;try{m=JSON.parse(r.headers.get('X-Meta')||'null')}catch(e){}
  const b=await r.blob();
  if(hold||tab!=2){busy=false;if(!hold)setTimeout(frame,50);return}   // Hold gedrueckt oder Seite gewechselt: Bild verwerfen
  lastMeta=m;lastBlob=b;const old=$('im2').src;$('im2').src=URL.createObjectURL(b);
  if(old&&old.startsWith('blob:'))setTimeout(()=>URL.revokeObjectURL(old),1500)}
 catch(e){busy=false;setTimeout(frame,1000)}}
['im1','im2'].forEach(i=>{const e=$(i);e.onload=()=>{busy=false;if(i=='im2')fit();setTimeout(frame,200)};e.onerror=()=>{busy=false;setTimeout(frame,1000)}});
function fit(){const c=$('cv'),im=$('im2');if(im.naturalWidth&&(c.width!=im.naturalWidth||c.height!=im.naturalHeight)){c.width=im.naturalWidth;c.height=im.naturalHeight}draw()}
const FC=()=>{const D=cfg.disp,c=$('cv');return{cx:(D.x+D.w/2)*c.width/1000,cy:(D.y+D.h/2)*c.height/1000,w:D.w*c.width/1000,h:D.h*c.height/1000,a:(D.rot||0)*Math.PI/180}};
const toF=p=>{const f=FC(),dx=p[0]-f.cx,dy=p[1]-f.cy,c=Math.cos(f.a),s=Math.sin(f.a);return[dx*c+dy*s+f.w/2,-dx*s+dy*c+f.h/2]};
const toW=q=>{const f=FC(),u=q[0]-f.w/2,v=q[1]-f.h/2,c=Math.cos(f.a),s=Math.sin(f.a);return[f.cx+u*c-v*s,f.cy+u*s+v*c]};
const RD=()=>{const f=FC();return{x:0,y:0,w:f.w,h:f.h}};
const RS=(d,s)=>{const R=RD(),r=cfg.disp.seg[d][s];return{x:r[0]*R.w/1000,y:r[1]*R.h/1000,w:r[2]*R.w/1000,h:r[3]*R.h/1000}};
const inside=(r,p)=>p[0]>=r.x&&p[0]<=r.x+r.w&&p[1]>=r.y&&p[1]<=r.y+r.h;
function DB(d){let x1=1e9,y1=1e9,x2=-1e9,y2=-1e9;for(let s=0;s<7;s++){const q=RS(d,s);x1=Math.min(x1,q.x);y1=Math.min(y1,q.y);x2=Math.max(x2,q.x+q.w);y2=Math.max(y2,q.y+q.h)}return{x:x1,y:y1,w:x2-x1,h:y2-y1}}
function corner(r,p,t){const cs=[[r.x,r.y],[r.x+r.w,r.y],[r.x,r.y+r.h],[r.x+r.w,r.y+r.h]];for(let i=0;i<4;i++)if(Math.hypot(cs[i][0]-p[0],cs[i][1]-p[1])<t)return cs[3-i];return null}
function draw(){const c=$('cv');if(!cfg||!c.width)return;const g=c.getContext('2d'),lw=Math.max(2,c.width/320);g.clearRect(0,0,c.width,c.height);
 const f=FC(),R=RD(),MK=(hold&&heldMeta&&heldMeta.mask)?heldMeta.mask:(st&&st.mask),LK=cfg.disp.lock;g.save();g.translate(f.cx,f.cy);g.rotate(f.a);g.translate(-f.w/2,-f.h/2);
 g.lineWidth=lw;g.strokeStyle=LK?'#6c6':'#0af';g.strokeRect(R.x,R.y,R.w,R.h);
 if(!LK)[[R.x,R.y],[R.x+R.w,R.y],[R.x,R.y+R.h],[R.x+R.w,R.y+R.h]].forEach(q=>{g.fillStyle='#0af';g.fillRect(q[0]-lw*2,q[1]-lw*2,lw*4,lw*4)});
 if(cfg.disp.seg.length)for(let d=0;d<cfg.disp.n;d++)for(let s=0;s<7;s++){if(!cfg.disp.seg[d]||!cfg.disp.seg[d][s])continue;const q=RS(d,s),on=MK&&((MK[d]>>s)&1);
  const issel=sel&&sel.d==d&&(sel.s==s||sel.s<0);g.lineWidth=issel?lw*1.5:lw*.7;g.strokeStyle=issel?'#ff0':on?'#0f0':'#f44';g.strokeRect(q.x,q.y,q.w,q.h)}
 g.restore()}
const P=e=>{const c=$('cv'),b=c.getBoundingClientRect();return[(e.clientX-b.left)*c.width/b.width,(e.clientY-b.top)*c.height/b.height]};
$('cv').onpointerdown=e=>{if(!cfg)return;if(cfg.disp.lock){msg('🔒 Rahmen gesperrt – Haken „Rahmen & Raster sperren“ lösen, um zu ändern');return}const c=$('cv'),pw=P(e),p=toF(pw),m=document.querySelector('[name=mode]:checked').value,t=Math.max(10,c.width/40),R=RD();c.setPointerCapture(e.pointerId);drag=null;
 if(m=='disp'){const f=corner(R,p,t);if(f)drag={k:'rect',t:'disp',fw:toW(f)};else if(inside(R,p))drag={k:'move',t:'disp',pw,o:{...cfg.disp}};else drag={k:'rect',t:'disp',fw:pw}}
 else if(m=='seg'){if(sel&&sel.s>=0){const f=corner(RS(sel.d,sel.s),p,t);if(f)drag={k:'rect',t:'seg',f}}
  if(!drag){let best=null,ba=1e18;cfg.disp.seg.forEach((D,d)=>D.forEach((r,s)=>{const q=RS(d,s);if(inside(q,p)&&q.w*q.h<ba){ba=q.w*q.h;best={d,s}}}));sel=best;if(best)drag={k:'move',t:'seg',p,o:cfg.disp.seg[best.d][best.s].slice()}}}
 else{let best=null,bd=1e18;cfg.disp.seg.forEach((D,d)=>{const b=DB(d);if(inside(b,p)){const q=Math.abs(p[0]-(b.x+b.w/2));if(q<bd){bd=q;best=d}}});
  sel=best==null?null:{d:best,s:-1};if(best!=null)drag={k:'move',t:'dig',p,o:cfg.disp.seg[best].map(r=>r.slice())}}
 draw()};
$('cv').onpointermove=e=>{if(!drag)return;const c=$('cv'),pw=P(e),p=toF(pw),W=c.width,H=c.height,D=cfg.disp,R=RD();
 if(drag.t=='disp'){if(drag.k=='rect'){const a=(D.rot||0)*Math.PI/180,cs=Math.cos(a),sn=Math.sin(a),dx=pw[0]-drag.fw[0],dy=pw[1]-drag.fw[1],qx=dx*cs+dy*sn,qy=-dx*sn+dy*cs;
   D.w=Math.max(5,Math.round(Math.abs(qx)*1000/W));D.h=Math.max(5,Math.round(Math.abs(qy)*1000/H));D.x=Math.round((pw[0]+drag.fw[0])/2*1000/W-D.w/2);D.y=Math.round((pw[1]+drag.fw[1])/2*1000/H-D.h/2)}
  else{D.x=Math.round(drag.o.x+(pw[0]-drag.pw[0])*1000/W);D.y=Math.round(drag.o.y+(pw[1]-drag.pw[1])*1000/H)}}
 else if(drag.t=='seg'){const r=D.seg[sel.d][sel.s];
  if(drag.k=='move'){r[0]=Math.round(drag.o[0]+(p[0]-drag.p[0])*1000/R.w);r[1]=Math.round(drag.o[1]+(p[1]-drag.p[1])*1000/R.h)}
  else{const a=[drag.f[0]*1000/R.w,drag.f[1]*1000/R.h],b=[p[0]*1000/R.w,p[1]*1000/R.h];
   r[0]=Math.round(Math.min(a[0],b[0]));r[1]=Math.round(Math.min(a[1],b[1]));r[2]=Math.max(1,Math.round(Math.abs(a[0]-b[0])));r[3]=Math.max(1,Math.round(Math.abs(a[1]-b[1])))}}
 else{const dx=(p[0]-drag.p[0])*1000/R.w,dy=(p[1]-drag.p[1])*1000/R.h;D.seg[sel.d].forEach((r,i)=>{r[0]=Math.round(drag.o[i][0]+dx);r[1]=Math.round(drag.o[i][1]+dy)})}
 draw()};
$('cv').onpointerup=()=>{if(drag){drag=null;push(false)}};
function ota(){const f=$('fw').files[0];if(!f)return;const x=new XMLHttpRequest();x.open('POST','/update');
 x.upload.onprogress=e=>$('otap').value=e.loaded/e.total*100;
 x.onload=()=>{msg(x.responseText+' – Neustart…');setTimeout(()=>location.reload(),10000)};
 x.setRequestHeader('Content-Type','application/octet-stream');x.send(f)}
async function imp(f){if(!f)return;const t=await f.text();try{JSON.parse(t)}catch(e){msg('Ungültige Datei');return}
 await fetch('/api/config?save=1',{method:'POST',headers:{'Content-Type':'application/json'},body:t});await load();msg('Importiert & gespeichert ✓')}
setInterval(async()=>{try{st=await (await fetch('/api/status')).json()}catch(e){return}
 $('stat').innerHTML=`${hold?'<b>⏸ HOLD</b> · ':''}Display: <b>${st.on?'AN':'AUS'}</b> · Helligkeit ${st.mean.toFixed(0)} / Kontrast ${st.sd.toFixed(0)} · Schwelle ${st.thr}${st.base>0?' · Ruhewert '+st.base.toFixed(0):''} · Erkannt: <code>${st.txt||'–'}</code> ${st.valid?'✓':''} · Letzter Wert: <b>${st.val||'–'}</b> · MQTT: ${st.mq?'verbunden':'–'} · ${st.ip} · v${st.fw} (${st.build}) · Laufzeit ${Math.floor(st.up/3600)} h · Heap ${(st.heap/1024)|0} kB (min ${(st.minheap/1024)|0} kB) · WLAN-Neuverbindungen ${st.wrc} · Neustart-Grund: ${st.rst}${st.cam?'':(st.camEn?' · ⚠ KAMERA FEHLER':' · Kamera aus')}`;
 if(tab==2)draw()},800);

// ---------- Hold, Zustands-Muster, Sperre ----------
const esc=t=>String(t).replace(/[&<>]/g,c=>c=='&'?'&amp;':c=='<'?'&lt;':'&gt;');
function syncUI(){document.querySelectorAll('[data-k]').forEach(e=>{const v=get(cfg,e.dataset.k);if(v===undefined)return;if(e.type=='checkbox')e.checked=!!v;else e.value=v;const o=e.nextElementSibling;if(o&&o.tagName=='B')o.textContent=e.value})}
function applyLockUI(){const L=!!(cfg&&cfg.disp&&cfg.disp.lock);
 document.querySelectorAll('#rotR,#rotN,#p2 [data-k],#p2 [name=mode],#autoBtn,#calBtn').forEach(e=>{const k=e.dataset.k;if(!k||/^disp\.(lay\.|n$)/.test(k))e.disabled=L});
 $('cv').style.cursor=L?'not-allowed':'crosshair';$('lockInfo').textContent=L?'gesperrt: Rahmen, Drehung und Raster sind geschützt':''}
function holdInfoUpd(){const m=heldMeta;$('holdInfo').textContent=hold?(m?`Gehaltenes Bild: Helligkeit ${m.mean}, Kontrast ${m.sd}, Ziffern ${m.nv}/${cfg.disp.n}, erkannt '${m.txt}' – die Erkennung sagt: ${m.on?'AN':'AUS'}`:'Gehaltenes Bild ohne Messwerte (noch kein Bild geladen oder Firmware zu alt)'):''}
function toggleHold(){hold=!hold;const b=$('holdBtn');
 if(hold){heldMeta=lastMeta;heldBlob=lastBlob;b.textContent='▶ Weiter';b.style.outline='3px solid #e80'}
 else{heldMeta=null;heldBlob=null;b.textContent='⏸ Hold';b.style.outline='';if(!busy)frame()}
 $('holdLbl').style.display=hold?'block':'none';holdInfoUpd();draw()}
function addSample(lbl){if(!hold||!heldMeta){msg('Kein gehaltenes Bild mit Messwerten – erst ⏸ Hold drücken');return}
 const m=heldMeta;samples.push({lbl,mean:+m.mean,sd:+m.sd,nv:+m.nv,ns:+m.ns,txt:String(m.txt||''),n:cfg.disp.n,url:heldBlob?URL.createObjectURL(heldBlob):''});
 renderSamples();const on=samples.filter(x=>x.lbl=='on').length;msg(`Muster gespeichert: ${lbl=='on'?'AN':'AUS'} (jetzt ${on} an / ${samples.length-on} aus)`)}
function smpDel(i){const s=samples[i];if(s&&s.url)URL.revokeObjectURL(s.url);samples.splice(i,1);renderSamples()}
function smpUndo(){if(samples.length)smpDel(samples.length-1)}
function smpClear(){samples.forEach(s=>{if(s.url)URL.revokeObjectURL(s.url)});samples=[];renderSamples();$('smpRes').textContent=''}
function renderSamples(){const on=samples.filter(s=>s.lbl=='on').length,off=samples.length-on;
 let h='<table><tr><th></th><th>Zustand</th><th>Helligkeit</th><th>Kontrast</th><th>Ziffern</th><th>Segm.</th><th>Text</th><th></th></tr>';
 samples.forEach((s,i)=>{h+=`<tr><td>${s.url?`<img src="${s.url}" style="height:34px">`:''}</td><td>${s.lbl=='on'?'🟢 AN':'⚫ AUS'}</td><td>${s.mean.toFixed(1)}</td><td>${s.sd.toFixed(1)}</td><td>${s.nv}/${s.n}</td><td>${s.ns}</td><td><code>${esc(s.txt.replace(/ /g,'·'))}</code></td><td><button onclick="smpDel(${i})">✕</button></td></tr>`});
 $('smpList').innerHTML=samples.length?h+'</table>':'<small>Noch keine Muster.</small>';$('smpCal').disabled=!(on>0&&off>0)}
// Trennt AN- von AUS-Mustern anhand der Messwerte und waehlt Verfahren + Wert der Display-an-Erkennung
function detectFit(samples,n){
 const ON=samples.filter(s=>s.lbl=='on'),OFF=samples.filter(s=>s.lbl=='off');
 const col=(a,k)=>a.map(s=>s[k]),mn=a=>Math.min(...a),mx=a=>Math.max(...a),all=samples;
 const rep=[],cand=[];
 const range=k=>Math.max(1e-9,mx(col(all,k))-mn(col(all,k)));
 // Verfahren 5: Ziffern erkannt (Schwelle N, Rueckfall bei N-1)
 {const hi=mn(col(ON,'nv')),lo=mx(col(OFF,'nv')),gap=hi-lo;
  rep.push(`Ziffern erkannt: AN ≥ ${hi}, AUS ≤ ${lo} von ${n} → ${gap>=2?'trennbar':'nicht trennbar'}`);
  if(gap>=2){const N=Math.min(hi,Math.max(1,Math.ceil((lo+2+hi)/2)));cand.push({mode:5,lvl:N,rel:(gap-1)/Math.max(1,range('nv')),txt:`Verfahren „Ziffern erkannt“, mindestens ${N} von ${n} Ziffern`})}}
 // Verfahren 0/2: Helligkeit bzw. Kontrast >= Wert (Rueckfall bei Wert-5)
 for(const [k,mode,name] of [['mean',0,'Helligkeit'],['sd',2,'Kontrast']]){
  const hi=mn(col(ON,k)),lo=mx(col(OFF,k)),gap=hi-lo;
  rep.push(`${name}: AN ≥ ${hi.toFixed(1)}, AUS ≤ ${lo.toFixed(1)} → ${gap>5?'trennbar':(gap>0?'zu knapp':'überlappen')}`);
  if(gap>5)cand.push({mode,lvl:Math.max(0,Math.min(255,Math.round((lo+5+hi+1)/2))),rel:(gap-5)/range(k),txt:`Verfahren „${name} ≥ Wert“, Wert ${Math.round((lo+5+hi+1)/2)}`})}
 // Verfahren 1: Helligkeit <= Wert (Anzeige dunkler, wenn an)
 {const hi=mx(col(ON,'mean')),lo=mn(col(OFF,'mean')),gap=lo-hi;
  rep.push(`Helligkeit (dunkler bei AN): AN ≤ ${hi.toFixed(1)}, AUS ≥ ${lo.toFixed(1)} → ${gap>5?'trennbar':'nein'}`);
  if(gap>5)cand.push({mode:1,lvl:Math.max(0,Math.min(255,Math.round((hi+lo-6)/2+0.5))),rel:(gap-5)/range('mean'),txt:`Verfahren „Helligkeit ≤ Wert“, Wert ${Math.round((hi+lo-6)/2+0.5)}`})}
 let best=null;const m5=cand.find(c=>c.mode==5);
 if(m5&&m5.rel>=0.5)best=m5;else for(const c of cand)if(!best||c.rel>best.rel)best=c;
 return{best,rep,cand}}
function calDetect(){const ON=samples.filter(s=>s.lbl=='on'),OFF=samples.filter(s=>s.lbl=='off');if(!ON.length||!OFF.length){msg('Je mindestens ein AN- und ein AUS-Muster nötig');return}
 const n=cfg.disp.n,r=detectFit(samples,n);let h='<b>Ergebnis</b><br>'+r.rep.map(t=>'• '+esc(t)).join('<br>')+'<br>';
 if(!r.best){h+='<br>⚠ <b>Keine saubere Trennung gefunden.</b> Prüfe: Sind die Bilder richtig als AN/AUS markiert? Sind in den AN-Bildern alle Ziffern lesbar (sonst erst 🎯 Auto-Kalibrierung der Ziffern)? Gleiches Licht in beiden Zuständen? Weitere Muster aufnehmen. Die Einstellungen wurden <b>nicht</b> geändert.'}
 else{cfg.det.mode=r.best.mode;cfg.det.lvl=r.best.lvl;syncUI();push(false);
  h+=`<br>✓ <b>Übernommen:</b> ${esc(r.best.txt)} (Trennreserve ${(r.best.rel*100).toFixed(0)} %). Noch nicht gespeichert – 💾 Speichern.`;
  if(ON.length<3||OFF.length<3)h+=`<br>Hinweis: Nur ${ON.length} AN- und ${OFF.length} AUS-Muster – für sichere Werte je 3 oder mehr aufnehmen, auch bei anderem Licht.`;
  if(r.best.rel<0.3)h+='<br>Hinweis: Geringe Reserve – die Erkennung kann im Betrieb schwanken.';
  const avgNv=ON.reduce((a,s)=>a+s.nv,0)/ON.length;if(avgNv<n*0.7)h+=`<br>Hinweis: In den AN-Bildern werden nur ${avgNv.toFixed(1)} von ${n} Ziffern gelesen. Wenn die Anzeige da komplett sichtbar war, erst die Ziffern kalibrieren (🎯 Auto-Kalibrierung).`}
 $('smpRes').innerHTML=h}
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
function setRot(v){if(cfg.disp.lock){msg('🔒 Gesperrt – erst den Haken „Rahmen & Raster sperren“ lösen');return}v=Math.max(-45,Math.min(45,Math.round(v*100)/100));cfg.disp.rot=v;$('rotR').value=v;$('rotN').value=v;draw();push(false)}
$('rotR').oninput=e=>setRot(+e.target.value);$('rotN').onchange=e=>setRot(+e.target.value);
// Auto-Kalibrierung fuer 7-Segment-LCD. G: Graustufen W*H, n Ziffern, opt.region {x,y,w,h} (Pixel) optional, opt.inv (dunkle Ziffern, Standard)
function calibrate(G, W, H, n, opt) {
  opt = opt || {}; const inv = opt.inv !== false; const deg = Math.PI / 180;
  const PAT = [63, 6, 91, 79, 102, 109, 125, 7, 127, 111];
  const rg = opt.region || { x: 0, y: 0, w: W, h: H };
  const cx = rg.x + rg.w / 2, cy = rg.y + rg.h / 2;           // Drehpunkt = Mitte der Suchregion
  const integ = (A) => { const S = new Float64Array((W + 1) * (H + 1)); for (let y = 0; y < H; y++) { let r = 0; for (let x = 0; x < W; x++) { r += A[y * W + x]; S[(y + 1) * (W + 1) + x + 1] = S[y * (W + 1) + x + 1] + r; } } return S; };
  const box = (S, x0, y0, x1, y1) => { x0 = Math.max(0, x0); y0 = Math.max(0, y0); x1 = Math.min(W, x1); y1 = Math.min(H, y1); if (x1 <= x0 || y1 <= y0) return [0, 0]; return [S[y1 * (W + 1) + x1] - S[y0 * (W + 1) + x1] - S[y1 * (W + 1) + x0] + S[y0 * (W + 1) + x0], (x1 - x0) * (y1 - y0)]; };
  // Dunkelkarte
  const SG = integ(G), R = Math.max(5, Math.round(rg.h / 10)), D = new Float32Array(W * H);
  for (let y = Math.max(0, rg.y | 0); y < Math.min(H, rg.y + rg.h); y++) for (let x = Math.max(0, rg.x | 0); x < Math.min(W, rg.x + rg.w); x++) {
    const [s, a] = box(SG, x - R, y - R, x + R + 1, y + R + 1); const bl = s / a, d = inv ? bl - G[y * W + x] : G[y * W + x] - bl; D[y * W + x] = d > 0 ? d : 0;
  }
  const rot = (src, th, F) => { const c = Math.cos(th), s = Math.sin(th), o = F ? new Float32Array(W * H) : new Uint8Array(W * H);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) { const sx = Math.round(cx + (x - cx) * c - (y - cy) * s), sy = Math.round(cy + (x - cx) * s + (y - cy) * c); if (sx >= 0 && sx < W && sy >= 0 && sy < H) o[y * W + x] = src[sy * W + sx]; } return o; };
  const x0r = Math.max(0, rg.x | 0), x1r = Math.min(W, Math.ceil(rg.x + rg.w)), y0r = Math.max(0, rg.y | 0), y1r = Math.min(H, Math.ceil(rg.y + rg.h));
  // Schwelle fuer "dunkler als Umgebung": ~20 % des 99-Perzentils im Suchbereich
  const vals = []; for (let y = y0r; y < y1r; y += 2) for (let x = x0r; x < x1r; x += 2) vals.push(D[y * W + x]); vals.sort((p, q) => p - q);
  const p99 = vals.length ? vals[Math.floor(vals.length * 0.99)] : 20, TB = Math.max(4, Math.min(14, 0.2 * p99));
  const Lmin = Math.max(6, Math.round(rg.h * 0.07)), Lmax = Math.round(rg.h * 0.36);
  // Profil: Summe der Laengen kurzer waagrechter dunkler Laeufe (Ziffernbalken); lange Rahmenlinien und senkrechte Striche zaehlen nicht
  const rowProf = (Dr) => { const P = new Float32Array(H); for (let y = y0r; y < y1r; y++) { let x = x0r, s = 0; while (x < x1r) { if (Dr[y * W + x] > TB) { let e = x; while (e < x1r && Dr[y * W + e] > TB) e++; const len = e - x; if (len >= Lmin && len <= Lmax) s += len; x = e; } else x++; } P[y] = s; } return P; };
  const sharp = (th) => { const P = rowProf(rot(D, th, true)); let s = 0; for (let y = y0r + 1; y < y1r; y++) { const d = P[y] - P[y - 1]; s += d * d; } return s; };
  // 1) Winkel
  let bth = 0, bs = -1;
  for (let a = -12; a <= 12; a += 1) { const s = sharp(a * deg); if (s > bs) { bs = s; bth = a * deg; } }
  for (const st of [0.25, 0.05]) { const c0 = bth; for (let k = -5; k <= 5; k++) { const th = c0 + k * st * deg, s = sharp(th); if (s > bs) { bs = s; bth = th; } } }
  let th = bth;
  // 2) Zeilen
  const Dr = rot(D, th, true); const P0 = rowProf(Dr); const Ps = new Float32Array(H); for (let y = 2; y < H - 2; y++) Ps[y] = (P0[y - 2] + 2 * P0[y - 1] + 3 * P0[y] + 2 * P0[y + 1] + P0[y + 2]) / 9;
  const pk = (y) => { let b = 0; for (let k = -3; k <= 3; k++) { const yy = y + k; if (yy >= 0 && yy < H && Ps[yy] > b) b = Ps[yy]; } return b; };
  let tb = { v: -1 }; const hmin = Math.max(24, rg.h * 0.4);
  for (let yt = y0r; yt < y1r; yt++) for (let yb = yt + Math.round(hmin); yb < y1r; yb++) { const a = pk(yt), c = pk(yb), m = pk((yt + yb) >> 1); const mn = Math.min(a, c, m); const v = (a + c + m) * (mn / Math.max(a, c, m)); if (v > tb.v) tb = { v, yt, yb }; }
  let yt = tb.yt, yb = tb.yb;
  const hw = (y) => { const h = pk(y) / 2; let u = y, d = y; while (u > y0r && Ps[u] > h) u--; while (d < y1r - 1 && Ps[d] > h) d++; return d - u; };
  let t = Math.max(3, Math.min(14, Math.round((hw(yt) + hw(yb)) / 2)));
  // 3) Modell
  let prm = { x0: x0r, p: rg.w / (n + 0.3), yt, yb, t, Lb: 0.62 * rg.w / (n + 0.3), d0: 0 };
  const build = (q) => { const ym = (q.yt + q.yb) / 2, rects = [];
    for (let i = 0; i < n; i++) { const xl = q.x0 + i * q.p + (q.dx ? q.dx[i] : 0), row = []; const d0i = q.d0 + (q.dd ? q.dd[i] : 0); const sh = (y) => d0i * (q.yb - y) / (q.yb - q.yt);
      const bar = (y) => { const w = Math.max(4, q.Lb * 0.6), h = Math.max(3, q.t + 1), xc = xl + q.Lb / 2 + sh(y); return [Math.round(xc - w / 2), Math.round(y - h / 2), Math.round(w), Math.round(h)]; };
      const vh = Math.max(5, (ym - q.yt) - 1.6 * q.t), vw = Math.max(3, q.t);
      const ver = (right, y) => { const xc = xl + (right ? q.Lb - q.t / 2 : q.t / 2) + sh(y); return [Math.round(xc - vw / 2), Math.round(y - vh / 2), Math.round(vw), Math.round(vh)]; };
      const yu = (q.yt + ym) / 2, yl = (ym + q.yb) / 2; row.push(bar(q.yt), ver(true, yu), ver(true, yl), bar(q.yb), ver(false, yl), ver(false, yu), bar(ym)); rects.push(row); }
    return rects; };
  let Gr, SR; const prep = (a) => { Gr = rot(G, a, false); SR = integ(Gr); };
  const contrast = (r) => { const [x, y, w, h] = r; const [si, ai] = box(SR, x, y, x + w, y + h); if (!ai) return 0; const mi = si / ai; let a1, a2;
    if (h > w) { const b = Math.max(2, w); a1 = box(SR, x - b, y, x, y + h); a2 = box(SR, x + w, y, x + w + b, y + h); } else { const b = Math.max(2, h); a1 = box(SR, x, y - b, x + w, y); a2 = box(SR, x, y + h, x + w, y + h + b); }
    const ao = a1[1] + a2[1]; if (!ao) return 0; const mo = (a1[0] + a2[0]) / ao; return inv ? mo - mi : mi - mo; };
  const dig = (rects, i) => { const c = rects[i].map(r => Math.min(35, Math.max(0, contrast(r)))); let b = -1e9; for (let k = 0; k < 10; k++) { let v = 0; for (let s = 0; s < 7; s++) v += ((PAT[k] >> s) & 1) ? c[s] : -1.5 * c[s]; if (v > b) b = v; } return b; };
  const obj = (q) => { const rects = build(q); let tot = 0; for (let i = 0; i < n; i++) { const c = rects[i].map(r => Math.min(35, Math.max(0, contrast(r)))); let sum = 0; for (const v of c) sum += v; let b = -1e9; for (let k = 0; k < 10; k++) { let v = 0; for (let s = 0; s < 7; s++) v += ((PAT[k] >> s) & 1) ? c[s] : -1.5 * c[s]; if (v > b) b = v; } tot += b; } return tot; };
  prep(th);
  // grobe Rastersuche
  let cur = -1e9; const hgt = yb - yt;
  for (let pf = 0.62; pf <= 1.12; pf += 0.03) { const p = rg.w / (n + 0.3) * pf; for (const Lf of [0.62, 0.75]) for (const d0 of [-0.08 * hgt, 0, 0.04 * hgt, 0.08 * hgt]) for (let xs = x0r - 14; xs < x1r - p * (n - 1) - 6; xs += 2) { const q = Object.assign({}, prm, { x0: xs, p, Lb: p * Lf, d0 }); const v = obj(q); if (v > cur) { cur = v; prm = q; } } }
  const dims = [['x0', 1, 5], ['p', 0.2, 6], ['yt', 1, 4], ['yb', 1, 4], ['t', 1, 2], ['Lb', 1, 4], ['d0', 1, 4]];
  const refine = (passes) => { for (let pass = 0; pass < passes; pass++) { let imp = false; for (const [k, st, ns] of dims) { const b = prm[k]; for (let j = -ns; j <= ns; j++) { if (!j) continue; const q = Object.assign({}, prm); q[k] = b + j * st; if (q.yb - q.yt < 20 || q.t < 2 || q.Lb < 6) continue; const v = obj(q); if (v > cur + 1e-6) { cur = v; prm = q; imp = true; } } } if (!imp) break; } };
  refine(5);
  // zweite Stufe: Zeilen, Pitch und Start gemeinsam in der Umgebung nachsuchen (korrigiert grobe Fehlstarts)
  { const b0 = Object.assign({}, prm); for (let dyt = -12; dyt <= 12; dyt += 3) for (let dyb = -12; dyb <= 12; dyb += 3) for (let dp = -2.4; dp <= 2.41; dp += 0.6) for (let dx = -10; dx <= 10; dx += 2) { const q = Object.assign({}, b0, { yt: b0.yt + dyt, yb: b0.yb + dyb, p: b0.p + dp, x0: b0.x0 + dx - dp * (n - 1) / 2 }); if (q.yb - q.yt < 30) continue; const v = obj(q); if (v > cur) { cur = v; prm = q; } } refine(5); }
  // Winkel mit Zielfunktion nachziehen
  for (const st of [0.5, 0.15]) { let bestA = th, bv = cur; for (let k = -3; k <= 3; k++) { if (!k) continue; const a = th + k * st * deg; prep(a); const v = obj(prm); if (v > bv) { bv = v; bestA = a; } } if (bestA !== th) { th = bestA; prep(th); cur = obj(prm); refine(3); } else prep(th); }
  prm.dx = new Array(n).fill(0); prm.dd = new Array(n).fill(0);
  for (let pass = 0; pass < 2; pass++) for (let i = 0; i < n; i++) { let bv = -1e9, bx = 0, bd = 0; for (let dx = -5; dx <= 5; dx++) for (let dd = -4; dd <= 4; dd += 2) { const q = Object.assign({}, prm, { dx: prm.dx.slice(), dd: prm.dd.slice() }); q.dx[i] = dx; q.dd[i] = dd; const v = dig(build(q), i); if (v > bv + 1e-9) { bv = v; bx = dx; bd = dd; } } prm.dx[i] = bx; prm.dd[i] = bd; }
  cur = obj(prm);
  const rects = build(prm); const C = rects.map(r => r.map(contrast));
  const flat = C.flat().map(v => Math.max(0, v)); const sv = flat.slice().sort((p, q) => p - q);
  // Schwelle an der groessten relativen Luecke zwischen "aus"-Haufen (~0) und "an"-Haufen
  let gi = 0, gr = 0; for (let i = 0; i + 1 < sv.length; i++) { const r = (sv[i + 1] + 2) / (sv[i] + 2); if (sv[i + 1] >= 6 && r > gr) { gr = r; gi = i; } }
  const lowv = sv[gi], highv = sv[gi + 1];
  const on = flat.filter(v => v >= highv), off = flat.filter(v => v <= lowv);
  const margin = highv - lowv;
  const delta = Math.max(3, Math.min(30, Math.round((lowv + highv) / 2)));
  let text = '', bad = 0; for (let i = 0; i < n; i++) { let m = 0; for (let s = 0; s < 7; s++) if (C[i][s] >= delta) m |= 1 << s; const k = PAT.indexOf(m); text += m === 0 ? ' ' : (k >= 0 ? String(k) : '?'); if (m === 0 || k < 0) bad++; }
  // Display-Rahmen = Bounding-Box der Segmente (rektifiziert) + Rand
  let bx0 = 1e9, by0 = 1e9, bx1 = -1e9, by1 = -1e9; for (const r of rects) for (const [x, y, w, h] of r) { bx0 = Math.min(bx0, x); by0 = Math.min(by0, y); bx1 = Math.max(bx1, x + w); by1 = Math.max(by1, y + h); }
  const mx = Math.round(prm.t * 1.5) + 2, my = Math.round(prm.t * 1.5) + 2; bx0 -= mx; bx1 += mx; by0 -= my; by1 += my;
  return { ok: true, theta: th / deg, prm, rects, C, text, bad, margin, delta, score: cur, frame: { x: bx0, y: by0, w: bx1 - bx0, h: by1 - by0 }, center: [cx, cy], rows: [yt, yb] };
}
async function grab(url){const r=await fetch(url);if(!r.ok)return null;const b=await createImageBitmap(await r.blob());const c=document.createElement('canvas');c.width=b.width;c.height=b.height;const g=c.getContext('2d',{willReadFrequently:true});g.drawImage(b,0,0);
 const d=g.getImageData(0,0,c.width,c.height).data,G=new Uint8Array(c.width*c.height);for(let i=0;i<G.length;i++)G[i]=d[i*4];return{G,W:c.width,H:c.height}}
function applyCal(r,W,H){const f=r.frame,cx=r.center[0],cy=r.center[1],a=r.theta*Math.PI/180,cs=Math.cos(a),sn=Math.sin(a);
 const qx=f.x+f.w/2-cx,qy=f.y+f.h/2-cy,sx=cx+qx*cs-qy*sn,sy=cy+qx*sn+qy*cs,D=cfg.disp;
 D.w=Math.round(f.w*1000/W);D.h=Math.round(f.h*1000/H);D.x=Math.round((sx-f.w/2)*1000/W);D.y=Math.round((sy-f.h/2)*1000/H);D.rot=Math.round(r.theta*100)/100;
 D.seg=r.rects.map(row=>row.map(q=>[Math.round((q[0]-f.x)*1000/f.w),Math.round((q[1]-f.y)*1000/f.h),Math.max(1,Math.round(q[2]*1000/f.w)),Math.max(1,Math.round(q[3]*1000/f.h))]));
 cfg.flt.mode=2;cfg.flt.delta=r.delta;cfg.flt.pct=50;cfg.flt.auto=true;cfg.flt.rad=Math.max(8,Math.round(r.prm.t*2.4*W/640));$('rotR').value=D.rot;$('rotN').value=D.rot;
 document.querySelectorAll('[data-k]').forEach(e=>{const v=get(cfg,e.dataset.k);if(v===undefined)return;if(e.type=='checkbox')e.checked=!!v;else e.value=v;const o=e.nextElementSibling;if(o&&o.tagName=='B')o.textContent=e.value});
 draw();push(false)}
let calBusy=false;
async function calib(){if(calBusy)return;if(cfg&&cfg.disp.lock){msg('🔒 Gesperrt – erst den Haken „Rahmen & Raster sperren“ lösen, dann kalibrieren');return}if(!cfg||!cfg.cam.en){msg('Kamera ist aus – bitte erst einschalten');return}
 calBusy=true;const n=cfg.disp.n,T0=Date.now();let best=null,bestImg=null,k=0;
 const score=r=>(r.bad==0?10000:0)-r.bad*1000+r.score;
 const D=cfg.disp,hint=(W,H,sc)=>{const cx=(D.x+D.w/2)*W/1000,cy=(D.y+D.h/2)*H/1000,w=D.w*W/1000*sc,h=D.h*H/1000*sc;return{x:Math.max(0,cx-w/2),y:Math.max(0,cy-h/2),w:Math.min(W,cx+w/2)-Math.max(0,cx-w/2),h:Math.min(H,cy+h/2)-Math.max(0,cy-h/2)}};
 try{
  const urls=[];try{const inf=await (await fetch('/api/diag/info')).json();if(inf.have)urls.push('/api/diag/image')}catch(e){}
  const dur=Math.max(5,Math.min(120,+($('calDur').value||30)))*1000;
  for(let i=0;i<400&&Date.now()-T0<dur;i++){
   const u=urls.length?urls.shift():'/snapshot.jpg?c='+Date.now();const im=await grab(u);if(!im)continue;k++;
   msg(`Kalibrierung: Bild ${k} wird ausgewertet… (bestes bisher: ${best?`'${best.text}', Reserve ${best.margin.toFixed(0)}`:'–'})`);await sleep(0);
   let r=calibrate(im.G,im.W,im.H,n,{region:hint(im.W,im.H,1.3),inv:cfg.flt.inv});if(r.bad>0){const r2=calibrate(im.G,im.W,im.H,n,{region:hint(im.W,im.H,1.0),inv:cfg.flt.inv});if(score(r2)>score(r))r=r2}
   if(!best||score(r)>score(best)){best=r;bestImg=im}
   if(best.bad==0&&best.margin>=12)break;await sleep(150)}
  if(best&&best.bad>0&&bestImg){msg('Kalibrierung: zweiter Versuch mit größerem Suchbereich…');await sleep(0);for(const sc of [1.7,0.85]){const r=calibrate(bestImg.G,bestImg.W,bestImg.H,n,{region:hint(bestImg.W,bestImg.H,sc),inv:cfg.flt.inv});if(score(r)>score(best))best=r}}
  if(!best){await sleep(300);msg('Kalibrierung fehlgeschlagen: kein Bild')}
  else{applyCal(best,bestImg.W,bestImg.H);const ok=best.bad==0&&best.margin>=8;
   await sleep(900);msg(`Kalibriert: '${best.text}', Drehung ${best.theta.toFixed(2)}°, Reserve ${best.margin.toFixed(0)}, Kontrast-Δ ${best.delta}`+(ok?' ✓ (noch nicht gespeichert – 💾 Speichern)':' – ⚠ nicht alle Ziffern gelesen: Rahmen näher um die Ziffern ziehen, Schärfe/Licht prüfen, dann nochmal'))}
 }catch(e){msg('Kalibrierung fehlgeschlagen: '+e)}
 calBusy=false}

function dl(b,n){const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download=n;document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),8000)}
function overlay(bmp,rs){const c=document.createElement('canvas');c.width=bmp.width;c.height=bmp.height;const g=c.getContext('2d');g.drawImage(bmp,0,0);
 const W=c.width,H=c.height,D=cfg.disp,lw=Math.max(1,W/400),fw=D.w*W/1000,fh=D.h*H/1000;
 g.save();g.translate((D.x+D.w/2)*W/1000,(D.y+D.h/2)*H/1000);g.rotate((D.rot||0)*Math.PI/180);g.translate(-fw/2,-fh/2);
 g.lineWidth=lw*2;g.strokeStyle='#0af';g.strokeRect(0,0,fw,fh);g.font=Math.max(9,Math.round(W/70))+'px sans-serif';
 for(let d=0;d<D.n&&D.seg[d];d++)for(let s=0;s<7;s++){const r=D.seg[d][s],q={x:r[0]*fw/1000,y:r[1]*fh/1000,w:r[2]*fw/1000,h:r[3]*fh/1000};
  const on=rs&&rs.mask&&((rs.mask[d]>>s)&1);g.lineWidth=lw;g.strokeStyle=on?'#0f0':'#f44';g.strokeRect(q.x,q.y,q.w,q.h);
  if(rs&&rs.frac&&rs.frac[d]){g.fillStyle='#ff0';g.fillText(rs.frac[d][s],q.x+1,q.y+q.h/2+4)}}
 g.restore();
 if(rs){g.fillStyle='#000';g.fillRect(0,0,W,Math.max(16,W/45));g.fillStyle='#fff';g.fillText(`Erkannt '${rs.txt}'  Schwelle ${rs.thr}  Helligkeit ${(rs.mean||0).toFixed(0)}  Kontrast ${(rs.sd||0).toFixed(0)}  Drehung ${(D.rot||0).toFixed(2)}°`,4,Math.max(12,W/60))}
 return new Promise(r=>c.toBlob(r,'image/png'))}
async function diag(){msg('Diagnose wird erstellt…');
 try{const info=await (await fetch('/api/diag/info')).json(),stNow=await (await fetch('/api/status')).json();
  const rl=await fetch('/snapshot.jpg?'+Date.now()),live=rl.ok?await rl.blob():null;
  let kept=null;if(info.have){const rk=await fetch('/api/diag/image');if(rk.ok)kept=await rk.blob()}
  const c=JSON.parse(JSON.stringify(cfg));c.wifi.pass='***';c.wifi.ssid='***';c.mqtt.pass='***';if(c.mqtt.user)c.mqtt.user='***';
  const pkg={erstellt:new Date().toISOString(),firmware:stNow.fw,build:stNow.build,
   hinweis:'jetzt = Livebild beim Klick (Messung: live_status). gespeichert = letztes Bild, bei dem die Anzeige als AN erkannt wurde (Messung: messung_gespeichert, Alter in s).',
   einstellungen:c,live_status:stNow,gespeichert_alter_s:info.have?info.age:null,messung_gespeichert:info.have?info.res:null};
  const t=new Date().toISOString().slice(0,19).replace(/[:T]/g,'-');let n=1;
  dl(new Blob([JSON.stringify(pkg,null,1)],{type:'application/json'}),`zaehlercam-diagnose-${t}.json`);
  if(live){await sleep(500);dl(live,`zaehlercam-jetzt-${t}.jpg`);await sleep(500);dl(await overlay(await createImageBitmap(live),stNow),`zaehlercam-jetzt-overlay-${t}.png`);n+=2}
  if(kept){await sleep(500);dl(kept,`zaehlercam-gespeichert-${t}.jpg`);await sleep(500);dl(await overlay(await createImageBitmap(kept),info.res),`zaehlercam-gespeichert-overlay-${t}.png`);n+=2}
  msg(`Diagnose gespeichert: ${n} Dateien ✓`+(live||kept?'':' (kein Bild – Kamera aus?)'))}
 catch(e){msg('Diagnose fehlgeschlagen: '+e)}}

// ---------- Aufnahme (Diagnose-Seite) ----------
const CRC=(()=>{const t=new Uint32Array(256);for(let n=0;n<256;n++){let c=n;for(let k=0;k<8;k++)c=c&1?0xEDB88320^(c>>>1):c>>>1;t[n]=c>>>0}return t})();
function crc32(u8){let c=0xFFFFFFFF;for(let i=0;i<u8.length;i++)c=CRC[(c^u8[i])&255]^(c>>>8);return(c^0xFFFFFFFF)>>>0}
function makeZip(files){const enc=new TextEncoder(),parts=[],cen=[];let off=0;
 const u16=v=>[v&255,(v>>8)&255],u32=v=>[v&255,(v>>8)&255,(v>>16)&255,(v>>>24)&255];
 for(const f of files){const nm=enc.encode(f.name),crc=crc32(f.data),sz=f.data.length;
  const lh=new Uint8Array([0x50,0x4b,3,4,...u16(20),...u16(0x0800),...u16(0),...u16(0),...u16(0x21),...u32(crc),...u32(sz),...u32(sz),...u16(nm.length),...u16(0)]);
  parts.push(lh,nm,f.data);
  cen.push(new Uint8Array([0x50,0x4b,1,2,...u16(20),...u16(20),...u16(0x0800),...u16(0),...u16(0),...u16(0x21),...u32(crc),...u32(sz),...u32(sz),...u16(nm.length),...u16(0),...u16(0),...u16(0),...u16(0),...u32(0),...u32(off)]),nm);
  off+=lh.length+nm.length+sz}
 let cs=0;for(const c of cen)cs+=c.length;
 const end=new Uint8Array([0x50,0x4b,5,6,...u16(0),...u16(0),...u16(files.length),...u16(files.length),...u32(cs),...u32(off),...u16(0)]);
 return new Blob([...parts,...cen,end],{type:'application/zip'})}
let recAbort=false,recRun=false;
async function recStart(){if(recRun)return;if(!cfg||!cfg.cam.en){msg('Kamera ist aus – bitte erst auf Seite 2 einschalten');return}
 const dur=Math.max(3,Math.min(180,+$('recDur').value||30)),gap=Math.max(0,Math.min(2000,+$('recGap').value||0)),qual=Math.max(10,Math.min(95,+$('recQ').value||75));
 const wantVid=$('recVid').checked&&typeof MediaRecorder!=='undefined',wantZip=$('recZip').checked;
 if(!wantVid&&!wantZip){msg('Bitte Video und/oder ZIP wählen');return}
 recRun=true;recAbort=false;$('recBtn').disabled=true;$('recStop').disabled=false;
 const frames=[],rows=[['bild','zeit_browser_ms','zeit_geraet_ms','alter_messung_ms','an','gueltig','helligkeit','kontrast','schwelle','ruhewert','zustand','ziffern_gueltig','segmente_an','text','maske_je_ziffer']];
 let canvas=null,g=null,mr=null,chunks=[],mrDone=null;const T0=performance.now();let n=0,lastOn=null,flips=0;
 try{
  while(!recAbort&&performance.now()-T0<dur*1000){
   const t1=performance.now();
   const r=await fetch(`/snapshot.jpg?m=1&q=${qual}&r=${Date.now()}`);if(!r.ok){await sleep(300);continue}
   let m={};try{m=JSON.parse(r.headers.get('X-Meta')||'{}')}catch(e){}
   const blob=await r.blob(),tb=performance.now()-T0;n++;
   if(wantZip)frames.push({name:`bilder/b${String(n).padStart(5,'0')}.jpg`,data:new Uint8Array(await blob.arrayBuffer()),t:tb});
   rows.push([n,Math.round(tb),m.t??'',m.age??'',m.on??'',m.valid??'',m.mean??'',m.sd??'',m.thr??'',m.base??'',m.st??'',m.nv??'',m.ns??'',m.txt??'',(m.mask||[]).join('|')]);
   if(lastOn!==null&&m.on!==undefined&&m.on!==lastOn)flips++;if(m.on!==undefined)lastOn=m.on;
   const bmp=await createImageBitmap(blob);
   if(wantVid){if(!canvas){canvas=document.createElement('canvas');canvas.width=bmp.width;canvas.height=bmp.height;g=canvas.getContext('2d');
     const mt=['video/webm;codecs=vp9','video/webm;codecs=vp8','video/webm'].find(x=>MediaRecorder.isTypeSupported(x));
     if(!mt){recInfoSet('Video nicht unterstützt – nur ZIP');}else{mr=new MediaRecorder(canvas.captureStream(15),{mimeType:mt,videoBitsPerSecond:3e6});mr.ondataavailable=e=>{if(e.data.size)chunks.push(e.data)};mrDone=new Promise(rs=>mr.onstop=rs);mr.start(500)}}
    if(g){g.drawImage(bmp,0,0);g.fillStyle='rgba(0,0,0,.7)';g.fillRect(0,0,canvas.width,34);g.font='15px monospace';g.fillStyle=m.on?'#6f6':'#f88';
     g.fillText(`#${n}  ${(tb/1000).toFixed(1)}s  ${m.on?'AN ':'aus'}  hell ${m.mean} kontr ${m.sd}  Ziffern ${m.nv}/${cfg.disp.n}  '${m.txt||''}'`,6,14);
     g.fillStyle='#fff';g.fillText(`Zustand ${m.st}  Schwelle ${m.thr}  Messung ${m.age} ms alt  Gerät ${m.t}`,6,29)}}
   $('recImg').src=URL.createObjectURL(blob);setTimeout(()=>URL.revokeObjectURL($('recImg').src),2000);
   $('recProg').value=Math.min(100,tb/10/dur);recInfoSet(`${n} Bilder, ${(tb/1000).toFixed(0)}/${dur} s, ${(n/Math.max(.1,tb/1000)).toFixed(1)} Bilder/s, Wechsel an↔aus: ${flips}`);
   const rest=gap-(performance.now()-t1);if(rest>0)await sleep(rest);else await sleep(0)}
  if(mr&&mr.state!=='inactive'){mr.stop();await mrDone}
  const t=new Date().toISOString().slice(0,19).replace(/[:T]/g,'-'),c=JSON.parse(JSON.stringify(cfg));c.wifi.pass='***';c.wifi.ssid='***';c.mqtt.pass='***';if(c.mqtt.user)c.mqtt.user='***';
  let stt0={};try{stt0=await (await fetch('/api/status')).json()}catch(e){}
  const csv=rows.map(r=>r.join(';')).join('\n'),fps=n/Math.max(.1,(performance.now()-T0)/1000);
  const info={erstellt:new Date().toISOString(),firmware:stt0.fw,build:stt0.build,dauer_s:dur,bilder:n,bilder_pro_s:+fps.toFixed(2),wechsel_an_aus:flips,jpeg_qualitaet:qual,einstellungen:c,status_am_ende:stt0,
   hinweis:'Messwerte je Bild stammen aus der Auswertung genau dieses Bildes (alter_messung_ms = 0; bei aelterer Firmware aus der letzten Auswertung). zustand: 0=aus,1=einschwingen,2=messen,3=fertig. maske: Bit0=a ... Bit6=g je Ziffer.'};
  let out=0;
  if(wantZip&&n){const enc=new TextEncoder(),files=[{name:'messwerte.csv',data:enc.encode('\ufeff'+csv)},{name:'info.json',data:enc.encode(JSON.stringify(info,null,1))},...frames.map(f=>({name:f.name,data:f.data}))];
   dl(makeZip(files),`zaehlercam-aufnahme-${t}.zip`);out++;await sleep(600)}
  if(chunks.length){dl(new Blob(chunks,{type:'video/webm'}),`zaehlercam-video-${t}.webm`);out++}
  if(!wantZip||!n){dl(new Blob(['\ufeff'+csv],{type:'text/csv'}),`zaehlercam-messwerte-${t}.csv`);out++}
  recInfoSet(`Fertig: ${n} Bilder (${fps.toFixed(1)}/s), ${flips} Wechsel an↔aus – ${out} Datei(en) gespeichert`);msg('Aufnahme gespeichert ✓')
 }catch(e){msg('Aufnahme fehlgeschlagen: '+e);recInfoSet('Fehler: '+e)}
 recRun=false;$('recBtn').disabled=false;$('recStop').disabled=true;$('recProg').value=0}
function recInfoSet(t){$('recInfo').textContent=t}
setInterval(()=>{if(busy&&Date.now()-tb>6000){busy=false;frame()}},2000);
show(0);load();
</script></body></html>)rawliteral";
