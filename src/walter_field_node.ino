/* ============================================================================
 *  LonePeak Radio LLC — WALTER FIELD NODE                                v1.12
 *  ESP32-S3 (DPTechnics Walter) cellular + DS18B20 + 0-25V battery + BME680
 *  + WiFi telemetry failover
 * ============================================================================
 *
 *  WHAT THIS BUILD DOES
 *  --------------------
 *    - Cellular dashboard (operator, band, signal, login-gated)
 *    - DS18B20 temperature probe on GPIO12
 *    - 0-25V battery sensor module on GPIO4 reading trailer battery
 *    - BME680 environmental sensor over I2C (SDA=GPIO8, SCL=GPIO9, 0x76):
 *      temperature, humidity, pressure, gas resistance, air-quality estimate
 *    - Sends JSON telemetry over UDP every interval to relay server.
 *      Primary path is CELLULAR. If cellular fails and WiFi STA is connected,
 *      sends the SAME packet over WiFi UDP -- relay treats both identically.
 *    - Payload includes: probe temp, CPU temp, signal, battery voltage,
 *      BME680 readings, uptime, operator, sequence number
 *
 *  WIRING
 *    DS18B20:
 *      Red    -> 3V3 on Walter
 *      Black  -> GND on Walter
 *      Yellow -> GPIO12
 *      4.7k resistor between Yellow and Red (REQUIRED)
 *
 *    Voltage sensor module (0-25V type with built-in 5:1 divider):
 *      S (signal)    -> GPIO4 on Walter
 *      + (VCC)       -> 3V3 on Walter
 *      - (GND)       -> GND on Walter
 *      Screw VCC     -> trailer battery POSITIVE (+)
 *      Screw GND     -> trailer battery NEGATIVE (-)
 *      The module internal divider is 30k:7.5k => /5 ratio.
 *      So 25V input becomes 5V output, 12V battery becomes 2.4V at GPIO4.
 *
 *    BME680 (I2C mode):
 *      VCC -> 3V3 on Walter
 *      GND -> GND on Walter
 *      SDA -> GPIO8 on Walter
 *      SCL -> GPIO9 on Walter
 *      CS  -> 3V3   (CS high selects I2C interface)
 *      SDO -> GND   (SDO low selects I2C address 0x76; SDO high = 0x77)
 *
 *  CALIBRATION
 *    Battery: tweak VBAT_DIVIDER_RATIO and/or ADC_CAL_FACTOR until they
 *    match a multimeter. Bump ADC_CAL_FACTOR up if reading LOW, down if HIGH.
 *
 *    BME680 gas resistance is RELATIVE not absolute. Air-quality estimate
 *    uses highest gas reading since boot as clean-air baseline. Needs burn-in
 *    period (minutes to half an hour). Treat as rough indicator only.
 *
 *  LIBRARIES (install via Arduino Library Manager)
 *    - OneWire              by Paul Stoffregen
 *    - DallasTemperature    by Miles Burton
 *    - Adafruit BME680      by Adafruit
 *    - Adafruit Unified Sensor by Adafruit (dependency of BME680)
 *
 *  CHANGELOG
 *    v1.12  WiFi telemetry failover. New helpers sendViaCellular() and
 *           sendViaWiFi(); sendTelemetry() prefers cellular and falls back
 *           to WiFi UDP when cellular is down or fails. New debug counters
 *           txCell/txWifi/txFail, lastTransport, uplink status on dashboard.
 *    v1.11  Added BME680 environmental sensor on I2C (SDA=GPIO8, SCL=GPIO9,
 *           addr 0x76). Non-blocking beginReading()/endReading() state machine.
 *           New JSON fields bme_t, bme_h, bme_p, bme_g. New dashboard card.
 *    v1.10  Added 0-25V trailer battery sensor on GPIO4. New vb field in JSON.
 *    v1.09  DS18B20 on GPIO12 (non-blocking, 12-bit).
 *    v1.08  Hourly UDP upload to relay server.
 *    v1.07b setRAT(AUTO) to clear stuck NB-IoT.
 * ============================================================================ */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <WalterModem.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define FW_VERSION "v1.12"

// ============================================================================
//  CONFIG — copy config_example.h to config.h and fill in your values
// ============================================================================
#include "config.h"

#define AP_AUTH_MODE  WIFI_AUTH_WPA2_WPA3_PSK
const int AP_MAX_CLIENTS = 2;

const int      VUSB_PIN             = 5;
const float    VUSB_DIVIDER         = 6.0f;
float          vusb                 = 0;

const int      ONEWIRE_PIN          = 12;
const uint8_t  DS_RES_BITS          = 12;
const uint32_t DS_READ_INTERVAL_MS  = 2000;

const int      VBAT_PIN             = 4;
const float    VBAT_DIVIDER_RATIO   = 5.0f;
const float    ADC_CAL_FACTOR       = 1.0f;
const uint32_t VBAT_READ_INTERVAL_MS = 2000;

const int      BME_SDA              = 8;
const int      BME_SCL              = 9;
const uint8_t  BME_I2C_ADDR         = 0x76;
const uint32_t BME_READ_INTERVAL_MS = 3000;
const uint32_t BME_REPROBE_MS       = 30000;

const uint32_t REFRESH_INTERVAL_MS  = 15000;
const uint32_t SEND_INTERVAL_MS     = 900000;   // 15 min production

const int      SOCKET_ID            = 1;
const uint16_t WIFI_UDP_LOCAL_PORT  = 20008;

// ============================================================================
//  STATE
// ============================================================================
WebServer           server(80);
WalterModem         modem;
WiFiUDP             wifiUdp;
OneWire             oneWire(ONEWIRE_PIN);
DallasTemperature   ds(&oneWire);
Adafruit_BME680     bme;

bool        modemStarted   = false;
bool        staIpLogged    = false;
bool        sendPrimed     = false;
uint32_t    bootMs         = 0;
uint32_t    lastRefresh    = 0;
uint32_t    lastSend       = 0;
uint32_t    lastSendMs     = 0;
uint32_t    sendSeq        = 0;
uint32_t    lastDsRequest  = 0;
uint32_t    lastVbatRead   = 0;
bool        dsAwaitingConv = false;
int         dsCount        = 0;
float       dsTempC        = NAN;
float       dsMinC         = NAN;
float       dsMaxC         = NAN;
uint32_t    dsLastGoodMs   = 0;
float       vbat           = NAN;
float       vbatMin        = NAN;
float       vbatMax        = NAN;
uint32_t    vbatLastReadMs = 0;
volatile int lastWifiEvent = -1;

String      lastTransport  = "none";
uint32_t    txCell         = 0;
uint32_t    txWifi         = 0;
uint32_t    txFail         = 0;

enum BmeState { BME_IDLE, BME_WAITING };
bool        bmePresent     = false;
BmeState    bmeState       = BME_IDLE;
uint32_t    bmeEndTime     = 0;
uint32_t    lastBmeTrigger = 0;
float       bmeTempC       = NAN;
float       bmeHum         = NAN;
float       bmePresHpa     = NAN;
float       bmeGasOhms     = NAN;
float       bmeGasMax      = NAN;
uint32_t    bmeLastGoodMs  = 0;
uint32_t    bmeErrCount    = 0;

struct CellStatus {
  bool    attached  = false;
  String  regState  = "not started";
  String  outcome   = "waiting for modem...";
  String  op        = "-";
  int     band      = 0;
  int     rssi      = 0;
  float   rsrp      = 0;
  float   rsrq      = 0;
} cell;

String   dlog;
String   lastSendResult = "none yet";
String   lastPayload    = "-";

// ============================================================================
//  LOGGING
// ============================================================================
void LOG(const String& s) {
  String line = "[t=" + String((millis() - bootMs) / 1000) +
                "|heap=" + String(ESP.getFreeHeap()) + "] " + s;
  Serial.println(line);
  dlog += line + "\n";
  if (dlog.length() > 6000) dlog = dlog.substring(dlog.length() - 6000);
}

#define DBG(x) LOG(String("[dbg] ") + (x))

String staStatusStr(int s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "0 IDLE";
    case WL_NO_SSID_AVAIL:   return "1 NO_SSID";
    case WL_SCAN_COMPLETED:  return "2 SCAN_DONE";
    case WL_CONNECTED:       return "3 CONNECTED";
    case WL_CONNECT_FAILED:  return "4 CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "5 CONNECTION_LOST";
    case WL_DISCONNECTED:    return "6 DISCONNECTED";
    default:                 return String(s);
  }
}

String regStateStr(WalterModemNetworkRegState st) {
  switch (st) {
    case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:      return "not searching";
    case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:    return "registered (home)";
    case WALTER_MODEM_NETWORK_REG_SEARCHING:          return "searching...";
    case WALTER_MODEM_NETWORK_REG_DENIED:             return "DENIED";
    case WALTER_MODEM_NETWORK_REG_UNKNOWN:            return "unknown";
    case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING: return "registered (roaming)";
    default:                                          return "other";
  }
}

String ago(uint32_t ms) {
  if (ms == 0) return "never";
  uint32_t s = (millis() - ms) / 1000;
  if (s < 60) return String(s) + "s ago";
  if (s < 3600) return String(s / 60) + "m " + String(s % 60) + "s ago";
  return String(s / 3600) + "h " + String((s % 3600) / 60) + "m ago";
}

String uplinkStatus() {
  bool c = cell.attached;
  bool w = (WiFi.status() == WL_CONNECTED);
  if (c && w) return "cellular + wifi";
  if (c)      return "cellular";
  if (w)      return "wifi only";
  return "NONE";
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  lastWifiEvent = (int)event;
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      LOG(String("[wifi] STA GOT IP ") + WiFi.localIP().toString()); break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      LOG(String("[wifi] STA DISCONNECTED reason=") +
          String(info.wifi_sta_disconnected.reason)); break;
    default: break;
  }
}

// ============================================================================
//  I2C BUS SCAN
// ============================================================================
void i2cScan() {
  LOG("[i2c] scanning bus (SDA=GPIO" + String(BME_SDA) +
      ", SCL=GPIO" + String(BME_SCL) + ")...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found++;
      char buf[40];
      snprintf(buf, sizeof(buf), "[i2c] Found device at 0x%02X", addr);
      LOG(String(buf));
    }
  }
  if (found == 0)
    LOG("[i2c] no devices found (check wiring / pull-ups)");
  else
    LOG("[i2c] scan complete, " + String(found) + " device(s)");
}

// ============================================================================
//  BME680 (non-blocking)
// ============================================================================
void bmeConfigure() {
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
}

void bmeBegin() {
  if (bme.begin(BME_I2C_ADDR)) {
    bmePresent = true;
    bmeConfigure();
    LOG("[BME680] Detected at 0x" + String(BME_I2C_ADDR, HEX));
  } else {
    bmePresent = false;
    LOG("[BME680] NOT FOUND at 0x" + String(BME_I2C_ADDR, HEX));
  }
}

float bmeAirQuality() {
  if (isnan(bmeGasOhms) || isnan(bmeHum)) return NAN;
  const float hum_ref = 40.0f;
  float hum_score;
  if (bmeHum >= 38.0f && bmeHum <= 42.0f)  hum_score = 25.0f;
  else if (bmeHum < 38.0f)                 hum_score = (bmeHum / hum_ref) * 25.0f;
  else                                     hum_score = ((100.0f - bmeHum) / (100.0f - hum_ref)) * 25.0f;
  if (hum_score < 0) hum_score = 0;
  if (hum_score > 25) hum_score = 25;
  float gas_ref = (isnan(bmeGasMax) || bmeGasMax <= 0) ? bmeGasOhms : bmeGasMax;
  float gas_score = (bmeGasOhms / gas_ref) * 75.0f;
  if (gas_score > 75) gas_score = 75;
  if (gas_score < 0)  gas_score = 0;
  return hum_score + gas_score;
}

String bmeAirQualityLabel() {
  float aq = bmeAirQuality();
  if (isnan(aq)) return "--";
  if (aq >= 80) return "EXCELLENT";
  if (aq >= 60) return "GOOD";
  if (aq >= 40) return "FAIR";
  if (aq >= 20) return "POOR";
  return "VERY POOR";
}

void bmeTick() {
  if (!bmePresent) {
    if (millis() - lastBmeTrigger > BME_REPROBE_MS) {
      lastBmeTrigger = millis();
      if (bme.begin(BME_I2C_ADDR)) {
        bmePresent = true;
        bmeConfigure();
        LOG("[BME680] appeared at 0x" + String(BME_I2C_ADDR, HEX));
      }
    }
    return;
  }
  if (bmeState == BME_IDLE) {
    if (millis() - lastBmeTrigger >= BME_READ_INTERVAL_MS) {
      uint32_t endT = bme.beginReading();
      lastBmeTrigger = millis();
      if (endT == 0) {
        bmeErrCount++;
        bmePresent = false;
        return;
      }
      bmeEndTime = endT;
      bmeState = BME_WAITING;
    }
  } else {
    if ((int32_t)(millis() - bmeEndTime) >= 0) {
      if (bme.endReading()) {
        bmeTempC   = bme.temperature;
        bmeHum     = bme.humidity;
        bmePresHpa = bme.pressure / 100.0f;
        bmeGasOhms = (float)bme.gas_resistance;
        bmeLastGoodMs = millis();
        if (isnan(bmeGasMax) || bmeGasOhms > bmeGasMax) bmeGasMax = bmeGasOhms;
      } else {
        bmeErrCount++;
      }
      bmeState = BME_IDLE;
    }
  }
}

// ============================================================================
//  BATTERY SENSOR
// ============================================================================
void vbatInit() {
  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  pinMode(VBAT_PIN, INPUT);
}

void vbatTick() {
  if (millis() - lastVbatRead < VBAT_READ_INTERVAL_MS) return;
  lastVbatRead = millis();
  uint32_t mv_sum = 0;
  for (int i = 0; i < 16; i++) mv_sum += analogReadMilliVolts(VBAT_PIN);
  float adc_v = (mv_sum / 16.0f) / 1000.0f;
  vbat = adc_v * VBAT_DIVIDER_RATIO * ADC_CAL_FACTOR;
  vbatLastReadMs = millis();
  if (isnan(vbatMin) || vbat < vbatMin) vbatMin = vbat;
  if (isnan(vbatMax) || vbat > vbatMax) vbatMax = vbat;
}

void vbatPrintSnapshot(const String& label) {
  if (isnan(vbat)) { LOG("[vbat " + label + "] no reading yet"); return; }
  LOG("[vbat " + label + "] current=" + String(vbat, 2) +
      "V  min=" + String(vbatMin, 2) + "V  max=" + String(vbatMax, 2) + "V");
}

// ============================================================================
//  DS18B20
// ============================================================================
void dsBegin() {
  ds.begin();
  dsCount = ds.getDeviceCount();
  ds.setResolution(DS_RES_BITS);
  ds.setWaitForConversion(false);
  LOG("[ds18b20] sensors: " + String(dsCount) +
      "  resolution: " + String(DS_RES_BITS) + " bits");
}

void dsTick() {
  if (dsCount == 0) {
    if (millis() - lastDsRequest > 30000) {
      lastDsRequest = millis();
      ds.begin();
      dsCount = ds.getDeviceCount();
      if (dsCount > 0) {
        ds.setResolution(DS_RES_BITS);
        ds.setWaitForConversion(false);
      }
    }
    return;
  }
  if (!dsAwaitingConv && (millis() - lastDsRequest > DS_READ_INTERVAL_MS)) {
    ds.requestTemperatures();
    lastDsRequest = millis();
    dsAwaitingConv = true;
    return;
  }
  uint16_t convMs = (DS_RES_BITS == 12) ? 750 :
                    (DS_RES_BITS == 11) ? 375 :
                    (DS_RES_BITS == 10) ? 188 : 94;
  if (dsAwaitingConv && (millis() - lastDsRequest >= convMs)) {
    float t = ds.getTempCByIndex(0);
    dsAwaitingConv = false;
    if (t != DEVICE_DISCONNECTED_C && t >= -55 && t <= 125) {
      dsTempC = t;
      dsLastGoodMs = millis();
      if (isnan(dsMinC) || t < dsMinC) dsMinC = t;
      if (isnan(dsMaxC) || t > dsMaxC) dsMaxC = t;
    }
  }
}

// ============================================================================
//  CELLULAR
// ============================================================================
bool lteConnect() {
  cell.outcome = "bringing up modem...";
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF))
    { cell.outcome = "ERROR: NO_RF"; return false; }
  if (!modem.definePDPContext(1, APN))
    { cell.outcome = "ERROR: PDP define"; return false; }
  modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC);
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL))
    { cell.outcome = "ERROR: radio enable"; return false; }
  cell.outcome = "searching for network...";
  for (int i = 0; i < 300; i++) {
    WalterModemNetworkRegState st = modem.getNetworkRegState();
    cell.regState = regStateStr(st);
    if (st == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
        st == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING) {
      cell.outcome = "ATTACHED";
      return true;
    }
    if (st == WALTER_MODEM_NETWORK_REG_DENIED)
      { cell.outcome = "DENIED"; return false; }
    for (int j = 0; j < 10; j++) {
      server.handleClient(); dsTick(); vbatTick(); bmeTick(); delay(100);
    }
  }
  cell.outcome = "TIMEOUT";
  return false;
}

void updateCellStatus() {
  WalterModemRsp rsp = {};
  if (modem.getRSSI(&rsp))            cell.rssi = rsp.data.rssi;
  if (modem.getSignalQuality(&rsp)) { cell.rsrp = rsp.data.signalQuality.rsrp;
                                      cell.rsrq = rsp.data.signalQuality.rsrq; }
  if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
    cell.op   = String(rsp.data.cellInformation.netName);
    cell.band = rsp.data.cellInformation.band;
  }
}

// ============================================================================
//  TELEMETRY
// ============================================================================
bool sendViaCellular(const String& p) {
  if (!modem.socketConfig(SOCKET_ID)) return false;
  modem.socketConfigSecure(SOCKET_ID, false);
  if (!modem.socketDial(SOCKET_ID, WALTER_MODEM_SOCKET_PROTO_UDP, UE_PORT, UE_HOST))
    return false;
  bool ok = modem.socketSend(SOCKET_ID, (uint8_t*)p.c_str(), p.length());
  modem.socketClose(SOCKET_ID);
  return ok;
}

bool sendViaWiFi(const String& p) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (wifiUdp.beginPacket(UE_HOST, UE_PORT) != 1) return false;
  wifiUdp.write((const uint8_t*)p.c_str(), p.length());
  return wifiUdp.endPacket() == 1;
}

void sendTelemetry() {
  bool cellOk = cell.attached;
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  if (!cellOk && !wifiOk) {
    lastSendResult = "skipped - no uplink";
    return;
  }
  vbatPrintSnapshot("send");
  float tc = temperatureRead();
  String p = "{";
  p += "\"tc\":"   + String(tc, 1)            + ",";
  if (!isnan(dsTempC))
    p += "\"ds\":" + String(dsTempC, 2)       + ",";
  if (!isnan(vbat))
    p += "\"vb\":" + String(vbat, 2)          + ",";
  if (bmePresent && !isnan(bmeTempC)) {
    p += "\"bme_t\":" + String(bmeTempC, 2)   + ",";
    p += "\"bme_h\":" + String(bmeHum, 1)     + ",";
    p += "\"bme_p\":" + String(bmePresHpa, 1) + ",";
    p += "\"bme_g\":" + String(bmeGasOhms, 0) + ",";
  }
  p += "\"rsrp\":" + String(cell.rsrp, 0)     + ",";
  p += "\"rssi\":" + String(cell.rssi)        + ",";
  p += "\"v\":"    + String(isnan(vbat)?0.0:vbat, 2) + ",";
  p += "\"up\":"   + String((millis()-bootMs)/1000)   + ",";
  p += "\"op\":\"" + cell.op                  + "\",";
  p += "\"seq\":"  + String(++sendSeq);
  p += "}";
  lastPayload = p;
  bool ok = false;
  if (cellOk) {
    ok = sendViaCellular(p);
    if (ok) { lastTransport = "cellular"; txCell++; }
    else if (wifiOk) {
      ok = sendViaWiFi(p);
      if (ok) { lastTransport = "wifi (cell-fallback)"; txWifi++; }
    }
  } else {
    ok = sendViaWiFi(p);
    if (ok) { lastTransport = "wifi"; txWifi++; }
  }
  if (ok) {
    lastSendResult = "sent " + String(p.length()) + " bytes via " + lastTransport;
    lastSendMs = millis();
  } else {
    txFail++;
    lastSendResult = "send FAILED (" + uplinkStatus() + ")";
    lastTransport  = "fail";
  }
}

// ============================================================================
//  DASHBOARD
// ============================================================================
String bars(float rsrp) {
  int b = 0;
  if      (rsrp >= -90)  b = 4;
  else if (rsrp >= -100) b = 3;
  else if (rsrp >= -110) b = 2;
  else if (rsrp < -110 && rsrp != 0) b = 1;
  String s;
  for (int i = 0; i < 4; i++) s += (i < b) ? "&#9608;" : "&#9617;";
  return s;
}

String signalLabel(float rsrp) {
  if (rsrp == 0)    return "--";
  if (rsrp >= -90)  return "EXCELLENT";
  if (rsrp >= -100) return "GOOD";
  if (rsrp >= -110) return "FAIR";
  return "POOR";
}

void handleRoot() {
  if (!server.authenticate(DASH_USER, DASH_PASS))
    return server.requestAuthentication();
  bool staUp = (WiFi.status() == WL_CONNECTED);
  String html = R"H(<!DOCTYPE html><html><head><meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<meta http-equiv='refresh' content='5'><title>Walter Field Node</title><style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a0a;color:#e0e0e0;font-family:'Courier New',monospace;padding:18px}
.wrap{max-width:520px;margin:0 auto}
h1{color:#00ff88;font-size:20px;letter-spacing:3px;text-transform:uppercase;text-align:center;margin-bottom:4px}
.sub{color:#555;font-size:11px;text-align:center;margin-bottom:18px}
.banner{border-radius:8px;padding:18px;text-align:center;margin-bottom:14px;font-size:24px;letter-spacing:2px;font-weight:bold}
.bgood{background:#0c2a18;border:1px solid #00ff88;color:#00ff88}
.bbad{background:#2a0c0c;border:1px solid #ff5555;color:#ff5555}
.bwait{background:#2a230c;border:1px solid #e8b07e;color:#e8b07e}
.card{background:#111;border:1px solid #222;border-radius:8px;padding:14px 16px;margin-bottom:12px}
.ctitle{font-size:10px;color:#00ff88;letter-spacing:2px;text-transform:uppercase;margin-bottom:10px}
.row{display:flex;justify-content:space-between;font-size:14px;margin:5px 0}
.row span:first-child{color:#888}
.big{font-size:22px;color:#fff}
.huge{font-size:34px;color:#00ff88;font-weight:bold}
.ok{color:#00ff88}.bad{color:#ff5555}.warn{color:#e8b07e}
.log{background:#000;border:1px solid #222;border-radius:6px;padding:12px;font-size:11px;
color:#9fefc4;white-space:pre-wrap;word-break:break-all;max-height:180px;overflow:auto}
a{color:#00ff88}
</style></head><body><div class='wrap'>
<h1>Walter Field Node</h1>
<div class='sub'>)H";
  html += FW_VERSION;
  html += R"H( &bull; DS18B20 + 0-25V + BME680 + cellular/wifi &bull; refresh 5s</div>)H";

  // Temperature
  html += "<div class='card'><div class='ctitle'>Temperature (DS18B20)</div>";
  if (dsCount == 0) {
    html += "<div class='row'><span class='bad'>NOT DETECTED</span></div>";
  } else if (isnan(dsTempC)) {
    html += "<div class='row'><span class='warn'>Reading...</span></div>";
  } else {
    float f = dsTempC * 9.0/5.0 + 32.0;
    html += "<div class='row'><span></span><span class='huge'>" + String(dsTempC, 2) + " &deg;C</span></div>";
    html += "<div class='row'><span></span><span class='big'>" + String(f, 2) + " &deg;F</span></div>";
    html += "<div class='row'><span>Min / Max</span><span>" +
            String(isnan(dsMinC)?0:dsMinC, 2) + " / " +
            String(isnan(dsMaxC)?0:dsMaxC, 2) + " &deg;C</span></div>";
    html += "<div class='row'><span>Last good</span><span>" + ago(dsLastGoodMs) + "</span></div>";
  }
  html += "</div>";

  // Battery
  html += "<div class='card'><div class='ctitle'>Trailer Battery (0-25V on GPIO4)</div>";
  if (isnan(vbat)) {
    html += "<div class='row'><span class='warn'>no reading yet</span></div>";
  } else {
    html += "<div class='row'><span></span><span class='huge'>" + String(vbat, 2) + " V</span></div>";
    html += "<div class='row'><span>Min / Max since boot</span><span>" +
            String(isnan(vbatMin)?0:vbatMin, 2) + " / " +
            String(isnan(vbatMax)?0:vbatMax, 2) + " V</span></div>";
    html += "<div class='row'><span>Last read</span><span>" + ago(vbatLastReadMs) + "</span></div>";
  }
  html += "</div>";

  // BME680
  html += "<div class='card'><div class='ctitle'>Environmental Sensor (BME680)</div>";
  if (!bmePresent) {
    html += "<div class='row'><span class='bad'>NOT DETECTED</span></div>";
  } else if (isnan(bmeTempC)) {
    html += "<div class='row'><span class='warn'>Reading...</span></div>";
  } else {
    float bf = bmeTempC * 9.0/5.0 + 32.0;
    html += "<div class='row'><span>Temperature</span><span class='big'>" +
            String(bmeTempC, 2) + " &deg;C / " + String(bf, 2) + " &deg;F</span></div>";
    html += "<div class='row'><span>Humidity</span><span class='big'>" + String(bmeHum, 1) + " %</span></div>";
    html += "<div class='row'><span>Pressure</span><span class='big'>" + String(bmePresHpa, 1) + " hPa</span></div>";
    html += "<div class='row'><span>Gas resistance</span><span>" +
            String(bmeGasOhms / 1000.0, 1) + " k&#8486;</span></div>";
    html += "<div class='row'><span>Air quality (est.)</span><span class='big'>" +
            bmeAirQualityLabel() + "</span></div>";
    html += "<div class='row'><span>Last good</span><span>" + ago(bmeLastGoodMs) + "</span></div>";
  }
  html += "</div>";

  // Cellular banner
  if (cell.attached) html += "<div class='banner bgood'>CELLULAR CONNECTED</div>";
  else if (cell.outcome.startsWith("searching") || cell.outcome.startsWith("bringing"))
    html += "<div class='banner bwait'>CONNECTING...</div>";
  else html += "<div class='banner bbad'>NOT CONNECTED</div>";

  // Cellular
  html += "<div class='card'><div class='ctitle'>Cellular</div>";
  html += "<div class='row'><span>Status</span><span class='" +
          String(cell.attached ? "ok'>ATTACHED" : "bad'>NOT ATTACHED") + "</span></div>";
  html += "<div class='row'><span>Operator</span><span class='big'>" + cell.op + "</span></div>";
  html += "<div class='row'><span>Band</span><span>" + String(cell.band) + "</span></div>";
  html += "<div class='row'><span>Reg state</span><span>" + cell.regState + "</span></div>";
  html += "</div>";

  // Signal
  html += "<div class='card'><div class='ctitle'>Signal</div>";
  html += "<div class='row'><span>Bars</span><span class='big'>" + bars(cell.rsrp) + "</span></div>";
  html += "<div class='row'><span>Quality</span><span>" + signalLabel(cell.rsrp) + "</span></div>";
  html += "<div class='row'><span>RSRP</span><span>" + String(cell.rsrp, 0) + " dBm</span></div>";
  html += "<div class='row'><span>RSSI</span><span>" + String(cell.rssi) + " dBm</span></div>";
  html += "<div class='row'><span>RSRQ</span><span>" + String(cell.rsrq, 0) + " dB</span></div>";
  html += "</div>";

  // Telemetry
  html += "<div class='card'><div class='ctitle'>Telemetry Upload</div>";
  html += "<div class='row'><span>Uplink</span><span class='" +
          String(uplinkStatus() == "NONE" ? "bad'>" : "ok'>") + uplinkStatus() + "</span></div>";
  html += "<div class='row'><span>Last transport</span><span>" + lastTransport + "</span></div>";
  html += "<div class='row'><span>TX cell/wifi/fail</span><span>" +
          String(txCell) + "/" + String(txWifi) + "/" + String(txFail) + "</span></div>";
  html += "<div class='row'><span>Sends (seq)</span><span>" + String(sendSeq) + "</span></div>";
  html += "<div class='row'><span>Last result</span><span>" + lastSendResult + "</span></div>";
  html += "<div class='row'><span>Last sent</span><span>" + ago(lastSendMs) + "</span></div>";
  html += "<div class='log' style='max-height:60px'>" + lastPayload + "</div>";
  html += "</div>";

  // Debug
  html += "<div class='card'><div class='ctitle'>Debug</div>";
  html += "<div class='row'><span>FW</span><span>" FW_VERSION "</span></div>";
  html += "<div class='row'><span>WiFi STA</span><span class='" +
          String(staUp ? "ok'>" : "warn'>") + staStatusStr(WiFi.status()) + "</span></div>";
  html += "<div class='row'><span>Uptime</span><span>" + String((millis()-bootMs)/1000) + " s</span></div>";
  html += "<div class='row'><span>Free heap</span><span>" + String(ESP.getFreeHeap()) + " B</span></div>";
  html += "<div class='row'><span>CPU temp</span><span>" + String(temperatureRead(), 1) + " &deg;C</span></div>";
  html += "</div>";

  html += "<div class='card'><div class='ctitle'>Log</div><div class='log'>" + dlog + "</div></div>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  bootMs = millis();
  Serial.begin(115200);
  delay(400);
  setCpuFrequencyMhz(240);
  LOG("=== WalterFieldNode " FW_VERSION " boot ===");
  vbatInit();
  dsBegin();
  Wire.begin(BME_SDA, BME_SCL);
  delay(50);
  i2cScan();
  bmeBegin();
  for (int i = 0; i < 3; i++) { vbatTick(); delay(50); }
  vbatPrintSnapshot("boot");
  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, AP_MAX_CLIENTS, false, AP_AUTH_MODE);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(STA_SSID, STA_PASS);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    server.handleClient(); dsTick(); vbatTick(); bmeTick(); delay(300);
  }
  wifiUdp.begin(WIFI_UDP_LOCAL_PORT);
  server.on("/", handleRoot);
  server.on("/bme", [](){ /* bme debug page */ server.send(200, "text/plain", "bme debug"); });
  server.begin();
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  server.handleClient();
  dsTick();
  vbatTick();
  bmeTick();
  if (!staIpLogged && WiFi.status() == WL_CONNECTED) {
    staIpLogged = true;
    LOG(String("[wifi] IP: http://") + WiFi.localIP().toString());
  }
  if (staIpLogged && WiFi.status() != WL_CONNECTED) staIpLogged = false;
  if (!modemStarted) {
    modemStarted = true;
    if (modem.begin(&Serial2)) {
      modem.setRAT(WALTER_MODEM_RAT_AUTO);
      delay(200);
      if (lteConnect()) {
        cell.attached = true;
        updateCellStatus();
        if (!sendPrimed) {
          lastSend = millis() - SEND_INTERVAL_MS + 30000;
          sendPrimed = true;
        }
      }
    } else {
      cell.outcome = "MODEM init failed";
    }
  }
  if (!sendPrimed && WiFi.status() == WL_CONNECTED) {
    lastSend = millis() - SEND_INTERVAL_MS + 30000;
    sendPrimed = true;
  }
  if (millis() - lastRefresh > REFRESH_INTERVAL_MS) {
    lastRefresh = millis();
    if (cell.attached) updateCellStatus();
  }
  bool uplinkUp = cell.attached || (WiFi.status() == WL_CONNECTED);
  if (uplinkUp && millis() - lastSend > SEND_INTERVAL_MS) {
    lastSend = millis();
    sendTelemetry();
  }
}
