/*
  ============================================================================
   PDU//NETWATCH  -  Multi-PDU power monitor for LilyGO T-Display-S3
                     (Arduino_GFX_Library edition, LED-dash style)
  ============================================================================
   Polls CyberPower + Tripp-Lite PDUs over SNMP (v2c) via WiFi and renders
   them as an 80s-supercar-style LED dashboard in a neon cyberpunk palette:
   slanted LED bargraph gauges, deviation meters and a sweeping scanner.

   BUTTONS
     KEY  (GPIO14, right) .. short press: next view
                             long press (1s): reset peak values
     BOOT (GPIO0,  left) ... short press: previous view
   VIEWS
     0 = overview of all PDUs + total load
     1 = CyberPower detail
     2 = Tripp-Lite detail
   The selected view is remembered across reboots.

   LOAD STATES (per PDU, thresholds in the PDU table below)
     >= warnW  (3600 W) .... yellow warning, scanner turns yellow
     >= alertW (5700 W) .... red flashing alert, fast red scanner,
                             screen border flashes on every view

   Library needed:  "GFX Library for Arduino" (Arduino_GFX_Library).
   Board: ESP32S3 Dev Module, USB CDC On Boot Enabled, 16MB flash,
          OPI PSRAM, partition 16M Flash (3MB APP/9.9MB FATFS).
  ============================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <Arduino_GFX_Library.h>
#include <vector>
#include <math.h>

// Custom types must be declared before any function. The Arduino IDE
// auto-generates function prototypes near the top of the sketch, and those
// prototypes need these types to already exist.
typedef std::vector<uint8_t> Bytes;
struct Reader { const uint8_t* p; size_t len; size_t pos; };
enum Align { A_LEFT, A_CENTER, A_RIGHT };

struct PduConfig {
  const char* name;        // shown on screen (keep <= 12 chars)
  IPAddress   ip;
  const char* community;
  const char* oids[3];     // watts, volts, hz
  float       scale[3];
  float       warnW;       // yellow warning at/above this
  float       alertW;      // red flashing alert at/above this
  uint32_t    timeoutMs;   // reply deadline per request
};

// Query modes, tried in order until one works:
//   0 = SNMPv2c, all 3 OIDs in one request
//   1 = SNMPv2c, one OID per request
//   2 = SNMPv1,  one OID per request
struct PduState {
  float    watts, volts, hz, peakW;
  int      failStreak;
  bool     pending;
  int32_t  reqId;
  uint32_t sentAt;
  uint32_t flashAt;
  uint32_t lastPollAt;
  int      mode;
  int      modeFails;
  int      step;           // which OID is in flight (single-OID modes)
  double   acc[3];
  bool     accOk[3];
  Bytes    oidEnc[3];
};

struct Button {
  uint8_t  pin;
  bool     down;
  bool     longFired;
  uint32_t changeAt;
  uint32_t downAt;
};

// ---------------------------------------------------------------- CONFIG ---
const char*     WIFI_SSID       = "YOUR_WIFI_SSID";
const char*     WIFI_PASS       = "YOUR_WIFI_PASSWORD";

const uint16_t  SNMP_PORT       = 161;
const uint16_t  SNMP_LOCAL_PORT = 16161;
const bool      SNMP_DEBUG      = true;   // verbose SNMP logging on Serial (115200)

const uint32_t  POLL_MS         = 2000;   // how often to query each PDU
const int       FAILS_TO_STALE  = 3;      // consecutive failures before "PDU LOST"
const uint32_t  LONG_PRESS_MS   = 1000;

const int NUM_PDUS = 2;
PduConfig PDUS[NUM_PDUS] = {
  { "CYBERPOWER", IPAddress(192, 168, 30, 21), "public",
    { "1.3.6.1.4.1.3808.1.1.3.2.3.1.1.7.1",       // Watts
      "1.3.6.1.4.1.3808.1.1.3.5.7.0",             // Volts (x10)
      "1.3.6.1.4.1.3808.1.1.3.5.8.0" },           // Hz    (x10)
    { 1.0f, 0.1f, 0.1f },
    3600.0f, 5700.0f, 1500 },

  { "TRIPP-LITE", IPAddress(192, 168, 30, 22), "public",
    { "1.3.6.1.4.1.850.1.1.3.2.2.1.1.9.1",        // Watts
      "1.3.6.1.4.1.850.1.1.3.2.3.1.2.1.4.1.1",    // Volts (x10)
      "1.3.6.1.4.1.850.1.1.3.2.3.1.2.1.3.1.1" },  // Hz    (x10)
    { 1.0f, 0.1f, 0.1f },
    3600.0f, 5700.0f, 3000 },                     // Tripp-Lite cards can be slow
};

// ------------------------------------------------------------- HARDWARE ---
#define PIN_POWER_ON  15   // must be HIGH for the LCD to run on battery
#define PIN_LCD_BL    38
#define PIN_LCD_RST    5
#define PIN_LCD_CS     6
#define PIN_LCD_DC     7
#define PIN_LCD_WR     8
#define PIN_LCD_RD     9
#define PIN_BTN_BOOT   0
#define PIN_BTN_KEY   14

Arduino_DataBus *bus = new Arduino_ESP32PAR8Q(
  PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_WR, PIN_LCD_RD,
  39, 40, 41, 42, 45, 46, 47, 48);                 // D0..D7

Arduino_GFX *panel = new Arduino_ST7789(
  bus, PIN_LCD_RST, 1 /* rotation: landscape */, true /* IPS */,
  170, 320, 35, 0, 35, 0);

Arduino_Canvas *gfx = new Arduino_Canvas(320, 170, panel);

// --------------------------------------------------------------- COLORS ---
#define C_BG       0x0802  // near-black violet
#define C_GRID     0x2006  // dark purple grid
#define C_CYAN     0x07FF
#define C_MAGENTA  0xF81F
#define C_YELLOW   0xFF61  // cyberpunk yellow
#define C_RED      0xF94A
#define C_DIMCYAN  0x02CD
#define C_DIMMAG   0x5808
#define C_GHOST    0x1885  // unlit 7-segment
#define C_ALERTBG  0x3000  // dark red flash fill
#define C_SCAN     0x2947
#define C_BLACK    0x0000

// ---------------------------------------------------------------- STATE ---
WiFiUDP     udp;
Preferences prefs;
PduState    st[NUM_PDUS];

const int NUM_VIEWS = NUM_PDUS + 1;     // overview + one detail per PDU
int view = 0;

Button btnKey  = { PIN_BTN_KEY,  false, false, 0, 0 };
Button btnBoot = { PIN_BTN_BOOT, false, false, 0, 0 };

int32_t  nextReqId    = 1000;
uint32_t lastFrame    = 0;
uint32_t lastWifiTry  = 0;
bool     wasConnected = false;
int      glitchFrames = 0;
int      scanY        = 0;
float    scannerT     = 0;     // 0..1 position of the sweeping scanner
int      scannerDir   = 1;

// =================================================== MINIMAL SNMP CLIENT ===
static void appendLen(Bytes& out, size_t len) {
  if (len < 0x80)        { out.push_back(len); }
  else if (len <= 0xFF)  { out.push_back(0x81); out.push_back(len); }
  else                   { out.push_back(0x82); out.push_back(len >> 8); out.push_back(len & 0xFF); }
}

static Bytes tlv(uint8_t tag, const Bytes& content) {
  Bytes out;
  out.push_back(tag);
  appendLen(out, content.size());
  out.insert(out.end(), content.begin(), content.end());
  return out;
}

static Bytes cat(std::initializer_list<Bytes> parts) {
  Bytes out;
  for (const Bytes& p : parts) out.insert(out.end(), p.begin(), p.end());
  return out;
}

static Bytes encInt(int32_t v) {
  uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
  int i = 0;
  while (i < 3 && ((b[i] == 0x00 && !(b[i + 1] & 0x80)) ||
                   (b[i] == 0xFF &&  (b[i + 1] & 0x80)))) i++;
  Bytes c(b + i, b + 4);
  return tlv(0x02, c);
}

static Bytes encOidBody(const char* s) {
  std::vector<uint32_t> ids;
  const char* p = s;
  while (*p) {
    ids.push_back(strtoul(p, (char**)&p, 10));
    if (*p == '.') p++;
  }
  Bytes c;
  if (ids.size() < 2) return c;
  c.push_back(ids[0] * 40 + ids[1]);
  for (size_t i = 2; i < ids.size(); i++) {
    uint32_t v = ids[i];
    uint8_t tmp[5]; int n = 0;
    do { tmp[n++] = v & 0x7F; v >>= 7; } while (v);
    while (n--) c.push_back(tmp[n] | (n ? 0x80 : 0x00));
  }
  return c;
}

// Builds a GET for OIDs [first, first+count) of PDU idx.
static Bytes buildGet(int idx, int32_t reqId, int first, int count, int version) {
  Bytes varbinds;
  for (int i = first; i < first + count; i++) {
    Bytes vb = tlv(0x30, cat({ tlv(0x06, st[idx].oidEnc[i]), Bytes{ 0x05, 0x00 } }));
    varbinds.insert(varbinds.end(), vb.begin(), vb.end());
  }
  const char* comm = PDUS[idx].community;
  Bytes community((const uint8_t*)comm, (const uint8_t*)comm + strlen(comm));
  Bytes pdu = tlv(0xA0, cat({ encInt(reqId), encInt(0), encInt(0), tlv(0x30, varbinds) }));
  return tlv(0x30, cat({ encInt(version), tlv(0x04, community), pdu }));
}

static const char* tagName(uint8_t t) {
  switch (t) {
    case 0x02: return "INTEGER";      case 0x04: return "OCTET STRING";
    case 0x05: return "NULL";         case 0x41: return "Counter32";
    case 0x42: return "Gauge32";      case 0x43: return "TimeTicks";
    case 0x46: return "Counter64";    case 0x80: return "noSuchObject";
    case 0x81: return "noSuchInstance"; case 0x82: return "endOfMibView";
    default:   return "unknown";
  }
}

static void hexDump(const uint8_t* p, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.printf("%02X%s", p[i], (i % 32 == 31) ? "\n" : " ");
  Serial.println();
}

static bool readTL(Reader& r, uint8_t& tag, size_t& l) {
  if (r.pos + 2 > r.len) return false;
  tag = r.p[r.pos++];
  uint8_t b = r.p[r.pos++];
  if (b & 0x80) {
    int n = b & 0x7F;
    if (n < 1 || n > 2 || r.pos + n > r.len) return false;
    l = 0;
    while (n--) l = (l << 8) | r.p[r.pos++];
  } else {
    l = b;
  }
  return r.pos + l <= r.len;
}

static int64_t decSigned(const uint8_t* p, size_t l) {
  uint64_t u = (l && (p[0] & 0x80)) ? ~0ULL : 0ULL;
  for (size_t i = 0; i < l; i++) u = (u << 8) | p[i];
  return (int64_t)u;
}

static uint64_t decUnsigned(const uint8_t* p, size_t l) {
  uint64_t u = 0;
  for (size_t i = 0; i < l; i++) u = (u << 8) | p[i];
  return u;
}

static bool decodeValue(uint8_t tag, const uint8_t* p, size_t l, double& out) {
  switch (tag) {
    case 0x02: out = (double)decSigned(p, l);   return l > 0 && l <= 8;   // INTEGER
    case 0x41:                                                            // Counter32
    case 0x42:                                                            // Gauge32
    case 0x43:                                                            // TimeTicks
    case 0x46: out = (double)decUnsigned(p, l); return l > 0 && l <= 9;   // Counter64
    case 0x04: {                                                          // OCTET STRING
      char buf[24]; size_t n = l < sizeof(buf) - 1 ? l : sizeof(buf) - 1;
      memcpy(buf, p, n); buf[n] = 0;
      out = atof(buf);
      return n > 0;
    }
    default:   return false;  // NULL, noSuchObject, noSuchInstance, endOfMibView...
  }
}

// Returns true if this packet is the reply to PDU idx's outstanding request.
static bool parseResponse(const uint8_t* buf, size_t n, int idx, double out[3], bool ok[3]) {
  Reader r{ buf, n, 0 }; uint8_t t; size_t l;
  if (!readTL(r, t, l) || t != 0x30) return false;              // message
  if (!readTL(r, t, l) || t != 0x02) return false; r.pos += l;  // version
  if (!readTL(r, t, l) || t != 0x04) return false; r.pos += l;  // community
  if (!readTL(r, t, l) || t != 0xA2) return false;              // GetResponse
  if (!readTL(r, t, l) || t != 0x02) return false;
  int32_t id = (int32_t)decSigned(r.p + r.pos, l); r.pos += l;
  if (id != st[idx].reqId) return false;
  if (!readTL(r, t, l) || t != 0x02) return false;
  int32_t err = (int32_t)decSigned(r.p + r.pos, l); r.pos += l;
  if (!readTL(r, t, l) || t != 0x02) return false;
  int32_t errIdx = (int32_t)decSigned(r.p + r.pos, l); r.pos += l;
  if (!readTL(r, t, l) || t != 0x30) return false;
  size_t listEnd = r.pos + l;

  while (r.pos < listEnd) {
    if (!readTL(r, t, l) || t != 0x30) return false;
    size_t vbEnd = r.pos + l;
    if (!readTL(r, t, l) || t != 0x06) return false;
    const uint8_t* oid = r.p + r.pos; size_t oidLen = l; r.pos += l;
    if (!readTL(r, t, l)) return false;
    const uint8_t* val = r.p + r.pos; size_t valLen = l;

    bool matched = false;
    for (int i = 0; i < 3; i++) {
      const Bytes& e = st[idx].oidEnc[i];
      if (e.size() == oidLen && memcmp(e.data(), oid, oidLen) == 0) {
        matched = true;
        double v;
        bool good = decodeValue(t, val, valLen, v);
        if (good) { out[i] = v; ok[i] = true; }
        if (SNMP_DEBUG) {
          if (good) Serial.printf("[SNMP] %s oid#%d %s = %.0f\n", PDUS[idx].name, i, tagName(t), v);
          else      Serial.printf("[SNMP] %s oid#%d returned %s (0x%02X) - OID not valid on this device?\n",
                                  PDUS[idx].name, i, tagName(t), t);
        }
      }
    }
    if (!matched && SNMP_DEBUG) Serial.printf("[SNMP] %s reply contained an unexpected OID\n", PDUS[idx].name);
    r.pos = vbEnd;
  }
  if (err != 0) {
    // error-status: 1 tooBig, 2 noSuchName, 3 badValue, 4 readOnly, 5 genErr
    Serial.printf("[SNMP] %s error-status %d (varbind %d)\n", PDUS[idx].name, err, errIdx);
  }
  return true;
}

// ================================================ NON-BLOCKING POLLING ===
static const char* MODE_NAMES[3] = { "v2c bulk", "v2c single-OID", "v1 single-OID" };

// Sends the request for the current mode/step of PDU i.
static void sendCurrent(int i, uint32_t now) {
  PduState& s = st[i];
  if (++nextReqId > 0x7FFFFFF0) nextReqId = 1000;
  s.reqId   = nextReqId;
  s.pending = true;
  s.sentAt  = now;

  int version = (s.mode == 2) ? 0 : 1;
  Bytes msg = (s.mode == 0) ? buildGet(i, s.reqId, 0, 3, version)
                            : buildGet(i, s.reqId, s.step, 1, version);

  int okTx = udp.beginPacket(PDUS[i].ip, SNMP_PORT);
  udp.write(msg.data(), msg.size());
  okTx = okTx && udp.endPacket();

  if (SNMP_DEBUG) {
    Serial.printf("[SNMP] -> %s %s (%s) req %d%s\n", PDUS[i].name,
                  PDUS[i].ip.toString().c_str(), MODE_NAMES[s.mode], s.reqId,
                  s.mode == 0 ? "" : (String(" oid#") + s.step).c_str());
    if (!okTx) Serial.println("[SNMP]    UDP send failed");
  }
}

static void startPoll(int i, uint32_t now) {
  PduState& s = st[i];
  s.step = 0;
  for (int k = 0; k < 3; k++) { s.acc[k] = NAN; s.accOk[k] = false; }
  sendCurrent(i, now);
}

static void pollFailed(int i) {
  PduState& s = st[i];
  s.failStreak++;
  Serial.printf("[%s] no usable reply (%d) via %s\n", PDUS[i].name, s.failStreak, MODE_NAMES[s.mode]);
  if (s.failStreak >= FAILS_TO_STALE) s.watts = s.volts = s.hz = NAN;

  // after 2 failed polls in a mode, try the next one
  if (++s.modeFails >= 2) {
    s.modeFails = 0;
    s.mode = (s.mode + 1) % 3;
    Serial.printf("[%s] switching to %s\n", PDUS[i].name, MODE_NAMES[s.mode]);
  }
}

static void finishPoll(int i) {
  PduState& s = st[i];
  s.pending = false;
  if (!s.accOk[0] && !s.accOk[1] && !s.accOk[2]) { pollFailed(i); return; }

  s.failStreak = 0;
  s.modeFails  = 0;
  s.flashAt    = millis();
  if (s.accOk[0]) { s.watts = s.acc[0] * PDUS[i].scale[0]; if (s.watts > s.peakW) s.peakW = s.watts; }
  if (s.accOk[1]) s.volts = s.acc[1] * PDUS[i].scale[1];
  if (s.accOk[2]) s.hz    = s.acc[2] * PDUS[i].scale[2];
  Serial.printf("[%s] %.0f W  %.1f V  %.1f Hz  (%s)\n", PDUS[i].name,
                s.watts, s.volts, s.hz, MODE_NAMES[s.mode]);
}

// Moves to the next OID in single-OID modes, or finishes the poll.
static void advance(int i, uint32_t now) {
  PduState& s = st[i];
  if (s.mode != 0 && ++s.step < 3) sendCurrent(i, now);
  else                             finishPoll(i);
}

static void serviceUdp() {
  uint8_t buf[512];
  while (udp.parsePacket() > 0) {
    IPAddress from = udp.remoteIP();
    int n = udp.read(buf, sizeof(buf));
    if (n <= 0) continue;

    bool claimed = false;
    for (int i = 0; i < NUM_PDUS; i++) {
      if (!st[i].pending) continue;
      double v[3] = { NAN, NAN, NAN };
      bool ok[3]  = { false, false, false };
      if (parseResponse(buf, n, i, v, ok)) {
        claimed = true;
        for (int k = 0; k < 3; k++) if (ok[k]) { st[i].acc[k] = v[k]; st[i].accOk[k] = true; }
        advance(i, millis());
        break;
      }
    }
    if (!claimed && SNMP_DEBUG) {
      Serial.printf("[SNMP] <- %d bytes from %s did not match a pending request:\n",
                    n, from.toString().c_str());
      hexDump(buf, n);
    }
  }
}

static void checkTimeouts(uint32_t now) {
  for (int i = 0; i < NUM_PDUS; i++) {
    PduState& s = st[i];
    if (s.pending && now - s.sentAt > PDUS[i].timeoutMs) {
      if (SNMP_DEBUG) Serial.printf("[SNMP] %s timeout (%s%s)\n", PDUS[i].name, MODE_NAMES[s.mode],
                                    s.mode == 0 ? "" : (String(" oid#") + s.step).c_str());
      advance(i, now);
    }
  }
}

// ================================================================= WIFI ===
static void handleWiFi(uint32_t now) {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    Serial.printf("[WiFi] connected, IP %s\n", WiFi.localIP().toString().c_str());
    udp.stop();
    udp.begin(SNMP_LOCAL_PORT);
    for (int i = 0; i < NUM_PDUS; i++) { st[i].pending = false; st[i].lastPollAt = 0; }
  }
  if (!connected && now - lastWifiTry > 10000) {
    lastWifiTry = now;
    Serial.println("[WiFi] reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  wasConnected = connected;
}

// ============================================================== BUTTONS ===
static void setView(int v) {
  view = (v + NUM_VIEWS) % NUM_VIEWS;
  glitchFrames = 4;                         // glitch burst on view change
  prefs.putUChar("view", view);
}

static void resetPeaks() {
  for (int i = 0; i < NUM_PDUS; i++) st[i].peakW = isnan(st[i].watts) ? 0 : st[i].watts;
  glitchFrames = 6;
  Serial.println("[UI] peaks reset");
}

static void onKeyShort()  { setView(view + 1); }
static void onBootShort() { setView(view - 1); }

static void handleButton(Button& b, uint32_t now, void (*onShort)(), void (*onLong)()) {
  bool pressed = digitalRead(b.pin) == LOW;
  if (pressed && !b.down && now - b.changeAt > 30) {
    b.down = true; b.longFired = false; b.changeAt = now; b.downAt = now;
  } else if (!pressed && b.down && now - b.changeAt > 30) {
    b.down = false; b.changeAt = now;
    if (!b.longFired && onShort) onShort();
  }
  if (b.down && !b.longFired && onLong && now - b.downAt > LONG_PRESS_MS) {
    b.longFired = true;
    onLong();
  }
}

// ========================================================= LOAD LOGIC ===
// -1 = offline, 0 = normal, 1 = warning, 2 = alert
static int loadLevel(int i) {
  const PduState& s = st[i];
  if (s.failStreak >= FAILS_TO_STALE || isnan(s.watts)) return -1;
  if (s.watts >= PDUS[i].alertW) return 2;
  if (s.watts >= PDUS[i].warnW)  return 1;
  return 0;
}

static uint16_t levelColor(int lv) {
  return lv == 2 ? C_RED : lv == 1 ? C_YELLOW : C_CYAN;
}

static float voltNominal(float v) { return v < 160 ? 120 : (v < 224 ? 208 : 240); }
static float hzNominal(float f)   { return f < 55 ? 50 : 60; }

// Warn when volts are >10% off nominal (120/208/240 auto-detected) or Hz >0.5 off.
static bool voltWarn(float v) { return !isnan(v) && fabsf(v - voltNominal(v)) / voltNominal(v) > 0.10f; }
static bool hzWarn(float f)   { return !isnan(f) && fabsf(f - hzNominal(f)) > 0.5f; }

// Deviation normalized so that +/-0.8 == warn limit, +/-1.0 == meter end.
static float voltDev(float v) {
  if (isnan(v)) return NAN;
  float nom = voltNominal(v);
  return (v - nom) / (nom * 0.10f * 1.25f);
}
static float hzDev(float f) {
  if (isnan(f)) return NAN;
  return (f - hzNominal(f)) / (0.5f * 1.25f);
}

// ======================================================= DRAW HELPERS ===
static uint16_t dimColor(uint16_t c, float f) {
  uint8_t r = ((c >> 11) & 31) * f;
  uint8_t g = ((c >> 5)  & 63) * f;
  uint8_t b = (c & 31) * f;
  return (r << 11) | (g << 5) | b;
}

// Built-in 6x8 font, scaled. (x, y) is the top edge of the text.
static void text(const char* s, int x, int y, uint8_t size, uint16_t col, Align a = A_LEFT) {
  int w = strlen(s) * 6 * size - size;
  if (a == A_CENTER) x -= w / 2;
  else if (a == A_RIGHT) x -= w;
  gfx->setTextSize(size);
  gfx->setTextColor(col);
  gfx->setCursor(x, y);
  gfx->print(s);
}

// --- Neon 7-segment renderer ------------------------------------------------
// segment bits: a=0 b=1 c=2 d=3 e=4 f=5 g=6
static const uint8_t SEG_DIGITS[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

static void drawSegs(int x, int y, int w, int h, int t, uint8_t m, uint16_t c) {
  const int g  = 1;
  const int hh = h / 2;
  const int hw = w - 2 * t - 2 * g;
  const int uy = y + t + g,          uh = hh - t / 2 - t - 2 * g;
  const int ly = y + hh + t / 2 + g, lh = (y + h - t) - (y + hh + t / 2) - 2 * g;

  if (m & 0x01) gfx->fillRect(x + t + g, y,              hw, t,  c);   // a
  if (m & 0x02) gfx->fillRect(x + w - t, uy,             t,  uh, c);   // b
  if (m & 0x04) gfx->fillRect(x + w - t, ly,             t,  lh, c);   // c
  if (m & 0x08) gfx->fillRect(x + t + g, y + h - t,      hw, t,  c);   // d
  if (m & 0x10) gfx->fillRect(x,         ly,             t,  lh, c);   // e
  if (m & 0x20) gfx->fillRect(x,         uy,             t,  uh, c);   // f
  if (m & 0x40) gfx->fillRect(x + t + g, y + hh - t / 2, hw, t,  c);   // g
}

// Numeric string right-aligned at rightX. Supports 0-9, '-', '.', ' '.
static void sevenSeg(const char* s, int rightX, int y, int w, int h, int t, int gap,
                     uint16_t on, uint16_t shadow, int shadowDx) {
  int total = 0;
  for (const char* p = s; *p; p++) total += (*p == '.') ? (t + gap) : (w + gap);
  int x = rightX - total + gap;

  for (const char* p = s; *p; p++) {
    char ch = *p;
    if (ch == '.') {
      if (shadowDx) gfx->fillRect(x + shadowDx, y + h - t, t, t, shadow);
      gfx->fillRect(x, y + h - t, t, t, on);
      x += t + gap;
      continue;
    }
    uint8_t m = 0;
    if (ch >= '0' && ch <= '9') m = SEG_DIGITS[ch - '0'];
    else if (ch == '-')         m = 0x40;
    drawSegs(x, y, w, h, t, 0x7F, C_GHOST);
    if (m && shadowDx) drawSegs(x + shadowDx, y, w, h, t, m, shadow);
    if (m)             drawSegs(x, y, w, h, t, m, on);
    x += w + gap;
  }
}

static void cutPanel(int x, int y, int w, int h, uint16_t col, int cut) {
  gfx->drawLine(x + cut, y, x + w, y, col);
  gfx->drawLine(x + w, y, x + w, y + h - cut, col);
  gfx->drawLine(x + w, y + h - cut, x + w - cut, y + h, col);
  gfx->drawLine(x + w - cut, y + h, x, y + h, col);
  gfx->drawLine(x, y + h, x, y + cut, col);
  gfx->drawLine(x, y + cut, x + cut, y, col);
}

// ===================================================== LED DASH GAUGES ===
// Slanted LED segment (parallelogram leaning right).
static void slantSeg(int x, int y, int w, int h, int slant, uint16_t c) {
  gfx->fillTriangle(x + slant, y, x + w + slant, y, x, y + h, c);
  gfx->fillTriangle(x + w + slant, y, x + w, y + h, x, y + h, c);
}

static uint16_t zoneColor(int i, float w) {
  return w >= PDUS[i].alertW ? C_RED : w >= PDUS[i].warnW ? C_YELLOW : C_CYAN;
}

// Horizontal "tach" bargraph: segments grow taller left -> right.
// Lit segments use zone colors; unlit are ghosted; peak-hold segment in magenta.
static void hBarGauge(int i, int x, int yBottom, int n, int pitch, int segW,
                      int hMin, int hMax, int slant, bool blink) {
  float maxW = PDUS[i].alertW * 1.1f;
  float w    = st[i].watts;
  int lit     = isnan(w) ? 0 : (int)roundf(constrain(w / maxW, 0.0f, 1.0f) * n);
  int peakIdx = st[i].peakW > 0 ? (int)roundf(constrain(st[i].peakW / maxW, 0.0f, 1.0f) * n) - 1 : -1;
  bool alert  = loadLevel(i) == 2;

  for (int s = 0; s < n; s++) {
    uint16_t zc = zoneColor(i, (s + 0.5f) / n * maxW);
    int h  = hMin + (hMax - hMin) * s / (n - 1);
    uint16_t col;
    if (s < lit)           col = (alert && !blink) ? dimColor(zc, 0.4f) : zc;
    else if (s == peakIdx) col = C_MAGENTA;
    else                   col = dimColor(zc, 0.18f);
    slantSeg(x + s * pitch, yBottom - h, segW, h, slant, col);
  }
}

// Vertical LED stack: slats widen toward the top.
static void vBarGauge(int i, int cx, int yBottom, int n, int pitch, int segH,
                      int wMin, int wMax, bool blink) {
  float maxW = PDUS[i].alertW * 1.1f;
  float w    = st[i].watts;
  int lit     = isnan(w) ? 0 : (int)roundf(constrain(w / maxW, 0.0f, 1.0f) * n);
  int peakIdx = st[i].peakW > 0 ? (int)roundf(constrain(st[i].peakW / maxW, 0.0f, 1.0f) * n) - 1 : -1;
  bool alert  = loadLevel(i) == 2;

  for (int s = 0; s < n; s++) {
    uint16_t zc = zoneColor(i, (s + 0.5f) / n * maxW);
    int sw = wMin + (wMax - wMin) * s / (n - 1);
    int y  = yBottom - (s + 1) * pitch + (pitch - segH);
    uint16_t col;
    if (s < lit)           col = (alert && !blink) ? dimColor(zc, 0.4f) : zc;
    else if (s == peakIdx) col = C_MAGENTA;
    else                   col = dimColor(zc, 0.18f);
    gfx->fillRect(cx - sw / 2, y, sw, segH, col);
  }
}

// Center-zero deviation meter: magenta nominal segment, lights spread outward.
static void devMeter(int x, int y, int n, int pitch, int segW, int h, float dev) {
  int center = n / 2;
  int litTo  = isnan(dev) ? 0 : (int)roundf(constrain(dev, -1.0f, 1.0f) * center);
  for (int s = 0; s < n; s++) {
    int off = s - center;
    float frac = fabsf((float)off) / center;
    uint16_t zc = frac > 0.8f ? C_RED : frac > 0.5f ? C_YELLOW : C_CYAN;
    bool lit = (off > 0 && off <= litTo) || (off < 0 && off >= litTo);
    uint16_t col = (off == 0) ? (isnan(dev) ? C_DIMMAG : C_MAGENTA)
                 : lit ? zc : dimColor(zc, 0.18f);
    gfx->fillRect(x + s * pitch, y, segW, h, col);
  }
}

// Sweeping scanner light with a fading trail.
static void scanner(int x, int y, int cells, int pitch, int cw, int h, int lv) {
  uint16_t col = lv == 2 ? C_RED : lv == 1 ? C_YELLOW : lv == -1 ? C_DIMMAG : C_MAGENTA;
  float head = scannerT * (cells - 1);
  for (int c = 0; c < cells; c++) {
    float behind = (head - c) * scannerDir;     // >0 means trailing the head
    float inten = 0;
    if (fabsf(head - c) < 0.6f)          inten = 1.0f;
    else if (behind > 0 && behind < 7)   inten = 1.0f - behind / 7.0f;
    uint16_t cc = inten > 0 ? dimColor(col, 0.15f + 0.85f * inten * inten)
                            : dimColor(col, 0.12f);
    gfx->fillRect(x + c * pitch, y, cw, h, cc);
  }
}

static void updateScanner(int lv) {
  float speed = lv == 2 ? 0.05f : lv == 1 ? 0.03f : lv == -1 ? 0.01f : 0.022f;
  scannerT += speed * scannerDir;
  if (scannerT >= 1) { scannerT = 1; scannerDir = -1; }
  if (scannerT <= 0) { scannerT = 0; scannerDir =  1; }
}

// Three-column pulsing voice-box display (used on the boot screen).
static void voiceBox(int cx, int cy, float amp) {
  const int colX[3]   = { cx - 34, cx, cx + 34 };
  const int maxSeg[3] = { 5, 7, 5 };
  for (int c = 0; c < 3; c++) {
    float a = constrain(amp * (c == 1 ? 1.0f : 0.75f) + random(-10, 11) / 100.0f, 0.0f, 1.0f);
    int lit = (int)roundf(a * maxSeg[c]);
    uint16_t on = c == 1 ? C_MAGENTA : C_CYAN;
    for (int s = 0; s < maxSeg[c]; s++) {
      uint16_t col = s < lit ? on : dimColor(on, 0.15f);
      gfx->fillRect(colX[c] - 11, cy - 3 - (s + 1) * 7 + 4, 22, 5, col);   // upper half
      gfx->fillRect(colX[c] - 11, cy + 3 + s * 7,          22, 5, col);   // lower half
    }
    gfx->fillRect(colX[c] - 11, cy - 2, 22, 4, on);                        // center bar
  }
}

// ======================================================== SCREEN PARTS ===
static void drawBackground() {
  gfx->fillScreen(C_BG);
  for (int x = 0; x < 320; x += 16) gfx->drawFastVLine(x, 20, 150, C_GRID);
  for (int y = 20; y < 170; y += 16) gfx->drawFastHLine(0, y, 320, C_GRID);
}

static void drawHeader(const char* title, const char* status, uint16_t sc,
                       bool dimStatus, int alertIdx, bool blink) {
  bool alertBar = alertIdx >= 0 && blink;
  uint16_t barCol = alertBar ? C_RED : C_MAGENTA;
  gfx->fillRect(0, 0, 200, 18, barCol);
  gfx->fillTriangle(200, 0, 214, 0, 200, 18, barCol);
  gfx->drawFastHLine(214, 18, 106, alertIdx >= 0 ? C_RED : C_CYAN);

  char t[40];
  if (alertBar) snprintf(t, sizeof(t), "!! OVERLOAD: %s", PDUS[alertIdx].name);
  else          snprintf(t, sizeof(t), "%s", title);
  text(t, 6, 5, 1, C_BLACK);
  text(t, 7, 5, 1, C_BLACK);                 // faux bold

  for (int i = 0; i < NUM_VIEWS; i++) {      // page indicator
    int x = 220 + i * 7;
    if (i == view) gfx->fillRect(x, 7, 4, 4, C_CYAN);
    else           gfx->drawRect(x, 7, 4, 4, C_DIMCYAN);
  }

  text(status, 292, 5, 1, dimStatus ? C_DIMMAG : sc, A_RIGHT);

  int bars = 0;                              // WiFi signal bars
  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    bars = rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : 1;
  }
  for (int i = 0; i < 4; i++) {
    int h = 4 + i * 3, x = 298 + i * 5, y = 16 - h;
    if (i < bars) gfx->fillRect(x, y, 3, h, C_CYAN);
    else          gfx->drawRect(x, y, 3, h, C_DIMCYAN);
  }
}

// ---- Detail view: one PDU as a full LED dash ----------------------------------
static void drawDetail(int i, bool glitch, bool blink) {
  const PduState& s = st[i];
  int lv = loadLevel(i);
  uint16_t col = levelColor(lv);

  if (lv == 2 && blink) gfx->fillRect(0, 20, 320, 88, C_ALERTBG);

  const char* lbl = lv == 2  ? "!! OVERLOAD // SHED LOAD"
                  : lv == 1  ? "HIGH LOAD // WARNING"
                  : lv == -1 ? "SIGNAL LOST"
                  :            "POWER DRAW";
  text(lbl, 10, 24, 1, lv == 0 ? C_YELLOW : (lv == -1 ? C_RED : col));

  if (lv == -1) {
    text("NO DATA", 228 + (glitch ? random(-3, 4) : 0), 34, 3, C_RED, A_CENTER);
    String ip = PDUS[i].ip.toString();
    char b[40]; snprintf(b, sizeof(b), "AWAITING %s", ip.c_str());
    text(b, 10, 46, 1, C_DIMCYAN);
  } else {
    char buf[12];
    float w = s.watts > 9999 ? 9999 : s.watts;
    snprintf(buf, sizeof(buf), "%4.0f", w);
    int jx = glitch ? random(-4, 5) : 0;
    sevenSeg(buf, 288, 24, 26, 40, 5, 5, col, C_MAGENTA, -3 + jx);
    text("W", 294, 50, 2, C_MAGENTA);

    char b[16];
    text("PEAK", 10, 38, 1, C_DIMCYAN);
    snprintf(b, sizeof(b), "%.0fW", s.peakW);
    text(b, 10, 48, 2, C_CYAN);

    text("LOAD", 96, 38, 1, C_DIMCYAN);
    snprintf(b, sizeof(b), "%.0f%%", s.watts / PDUS[i].alertW * 100.0f);
    text(b, 96, 48, 2, col);
  }

  // main LED tach gauge
  hBarGauge(i, 16, 94, 36, 8, 6, 12, 26, 3, blink);
  float maxW = PDUS[i].alertW * 1.1f;
  for (int k = 0; k * 1000 <= maxW; k++) {
    char b[12];
    if (k == 0) strcpy(b, "0"); else snprintf(b, sizeof(b), "%dk", k);
    int tx = 16 + (int)(k * 1000 / maxW * 288);
    text(b, tx + 3, 98, 1, C_DIMCYAN, A_CENTER);
  }

  // deviation meters
  bool vw = voltWarn(s.volts), hw = hzWarn(s.hz);
  cutPanel(2, 110, 316, 42, (vw || hw) ? C_RED : C_MAGENTA, 6);

  char vb[12], hb[12];
  if (isnan(s.volts)) strcpy(vb, " --.-"); else snprintf(vb, sizeof(vb), "%5.1f", s.volts);
  if (isnan(s.hz))    strcpy(hb, " --.-"); else snprintf(hb, sizeof(hb), "%5.1f", s.hz);

  text("VOLTS", 10, 117, 1, C_YELLOW);
  devMeter(52, 115, 25, 6, 4, 10, voltDev(s.volts));
  sevenSeg(vb, 290, 113, 9, 14, 2, 2, vw ? C_RED : C_CYAN, 0, 0);
  text("V", 296, 117, 1, C_MAGENTA);

  text("FREQ", 10, 136, 1, C_YELLOW);
  devMeter(52, 134, 25, 6, 4, 10, hzDev(s.hz));
  sevenSeg(hb, 290, 132, 9, 14, 2, 2, hw ? C_RED : C_CYAN, 0, 0);
  text("HZ", 296, 136, 1, C_MAGENTA);

  scanner(2, 157, 40, 8, 6, 9, lv);
}

// ---- Overview: LED stack per PDU + total ----------------------------------------
static void drawMiniPanel(int i, int x, int y, int w, int h, bool glitch, bool blink) {
  const PduState& s = st[i];
  int lv = loadLevel(i);
  uint16_t col = levelColor(lv);

  if (lv == 2 && blink) gfx->fillRect(x + 1, y + 1, w - 1, h - 1, C_ALERTBG);
  cutPanel(x, y, w, h, lv >= 1 ? col : C_MAGENTA, 8);
  text(PDUS[i].name, x + 7, y + 5, 1, C_YELLOW);

  if (millis() - s.flashAt < 200) gfx->fillRect(x + w - 11, y + 5, 5, 5, C_YELLOW);
  else                            gfx->drawRect(x + w - 11, y + 5, 5, 5, C_DIMCYAN);

  // vertical LED stack (bottom-left of panel)
  vBarGauge(i, x + 20, y + h - 6, 20, 5, 4, 14, 24, blink);

  const int rx = x + 38;                     // readout column start
  if (lv == -1) {
    text("NO DATA", rx + (w - 38) / 2 - 4 + (glitch ? random(-3, 4) : 0), y + 40, 2, C_RED, A_CENTER);
    String ip = PDUS[i].ip.toString();
    text(ip.c_str(), rx + (w - 38) / 2 - 4, y + 62, 1, C_DIMCYAN, A_CENTER);
    return;
  }

  char buf[12];
  float wv = s.watts > 9999 ? 9999 : s.watts;
  snprintf(buf, sizeof(buf), "%4.0f", wv);
  int jx = glitch ? random(-3, 4) : 0;
  sevenSeg(buf, x + w - 18, y + 18, 18, 32, 4, 4, col, C_MAGENTA, -2 + jx);
  text("W", x + w - 15, y + 36, 2, C_MAGENTA);

  const char* tag = lv == 2 ? "OVERLOAD" : lv == 1 ? "HI LOAD" : "NOMINAL";
  text(tag, rx + 2, y + 57, 1, lv == 0 ? C_DIMCYAN : col);

  char vb[12], hb[12], pb[16];
  if (isnan(s.volts)) strcpy(vb, "--.-"); else snprintf(vb, sizeof(vb), "%.1f", s.volts);
  if (isnan(s.hz))    strcpy(hb, "--.-"); else snprintf(hb, sizeof(hb), "%.1f", s.hz);

  text("V", rx + 2, y + 73, 1, C_YELLOW);
  devMeter(rx + 12, y + 72, 11, 5, 3, 9, voltDev(s.volts));
  text(vb, x + w - 6, y + 73, 1, voltWarn(s.volts) ? C_RED : C_CYAN, A_RIGHT);

  text("HZ", rx + 2, y + 89, 1, C_YELLOW);
  devMeter(rx + 16, y + 88, 9, 5, 3, 9, hzDev(s.hz));
  text(hb, x + w - 6, y + 89, 1, hzWarn(s.hz) ? C_RED : C_CYAN, A_RIGHT);

  snprintf(pb, sizeof(pb), "PK %.0fW", s.peakW);
  text(pb, rx + 2, y + 106, 1, C_DIMCYAN);
}

static void drawOverview(bool glitch, bool blink, int worstLv) {
  drawMiniPanel(0, 2,   22, 155, 128, glitch, blink);
  drawMiniPanel(1, 163, 22, 155, 128, glitch, blink);

  float total = 0; int online = 0;
  for (int i = 0; i < NUM_PDUS; i++) {
    if (loadLevel(i) >= 0) { total += st[i].watts; online++; }
  }

  text("TOTAL", 4, 158, 1, online == NUM_PDUS ? C_YELLOW : C_RED);
  char buf[12];
  if (online == 0) strcpy(buf, "-----");
  else             snprintf(buf, sizeof(buf), "%5.0f", total > 99999 ? 99999 : total);
  sevenSeg(buf, 104, 155, 9, 13, 2, 2, C_CYAN, 0, 0);
  text("W", 108, 159, 1, C_MAGENTA);

  scanner(122, 157, 28, 7, 5, 9, worstLv);
}

static void drawGlitchSlices() {
  for (int i = 0; i < 3; i++) {
    int y = random(20, 168), x = random(0, 280), w = random(10, 60);
    gfx->fillRect(x, y, w, 2, random(2) ? C_MAGENTA : C_CYAN);
  }
}

static void drawFrame() {
  uint32_t now = millis();
  bool blink = (now / 250) % 2;

  if (glitchFrames > 0) glitchFrames--;
  else if (random(100) < 2) glitchFrames = random(2, 5);
  bool glitch = glitchFrames > 0;

  int alertIdx = -1, worstLv = -1; bool anyWarn = false, anyLost = false;
  for (int i = 0; i < NUM_PDUS; i++) {
    int lv = loadLevel(i);
    if (lv == 2 && alertIdx < 0) alertIdx = i;
    if (lv == 1)  anyWarn = true;
    if (lv == -1) anyLost = true;
    if (lv > worstLv) worstLv = lv;
  }

  char title[40];
  if (view == 0) snprintf(title, sizeof(title), "NETWATCH//ALL PDUS");
  else           snprintf(title, sizeof(title), "NETWATCH//%s", PDUS[view - 1].name);

  const char* status; uint16_t sc; bool flashStatus = false;
  if (WiFi.status() != WL_CONNECTED) { status = "NO WIFI";  sc = C_RED; flashStatus = true; }
  else if (alertIdx >= 0)            { status = "OVERLOAD"; sc = C_RED; flashStatus = true; }
  else {
    bool lost = view == 0 ? anyLost : loadLevel(view - 1) == -1;
    bool warn = view == 0 ? anyWarn : loadLevel(view - 1) == 1;
    if (lost)      { status = "PDU LOST"; sc = C_RED; flashStatus = true; }
    else if (warn) { status = "HI LOAD";  sc = C_YELLOW; }
    else           { status = "ONLINE";   sc = C_CYAN; }
  }

  updateScanner(view == 0 ? worstLv : loadLevel(view - 1));

  drawBackground();
  drawHeader(title, status, sc, flashStatus && blink, alertIdx, blink);
  if (view == 0) drawOverview(glitch, blink, worstLv);
  else           drawDetail(view - 1, glitch, blink);

  scanY = (scanY + 3) % 170;
  gfx->drawFastHLine(0, scanY, 320, C_SCAN);

  if (glitch) drawGlitchSlices();

  // any PDU in alert -> flashing red screen border on every view
  if (alertIdx >= 0 && blink) {
    for (int k = 0; k < 3; k++) gfx->drawRect(k, k, 320 - 2 * k, 170 - 2 * k, C_RED);
  }
  gfx->flush();
}

static void drawBoot(int tick) {
  gfx->fillScreen(C_BG);
  for (int y = 0; y < 170; y += 3) gfx->drawFastHLine(0, y, 320, C_GRID);

  float amp = 0.5f + 0.5f * sinf(tick * 0.35f) * sinf(tick * 0.11f);
  voiceBox(160, 58, fabsf(amp));

  int jx = (tick % 9 == 0) ? random(-3, 4) : 0;
  text("JACKING IN", 160 + jx - 2, 115, 2, C_MAGENTA, A_CENTER);
  text("JACKING IN", 160,          114, 2, C_CYAN,    A_CENTER);

  char buf[48];
  snprintf(buf, sizeof(buf), "NET: %s", WIFI_SSID);
  text(buf, 160, 136, 1, C_YELLOW, A_CENTER);

  updateScanner(0);
  scanner(2, 155, 40, 8, 6, 9, 0);
  gfx->flush();
}

// ================================================================ SETUP ===
void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);
  pinMode(PIN_BTN_KEY,  INPUT_PULLUP);

  Serial.begin(115200);
  randomSeed(esp_random());

  prefs.begin("netwatch", false);
  view = prefs.getUChar("view", 0) % NUM_VIEWS;

  if (!gfx->begin()) {
    Serial.println("[GFX] begin() failed - framebuffer allocation? Enable OPI PSRAM.");
  }
  gfx->setTextWrap(false);
  gfx->fillScreen(C_BLACK);
  gfx->flush();

  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);

  for (int i = 0; i < NUM_PDUS; i++) {
    st[i].watts = st[i].volts = st[i].hz = NAN;
    st[i].peakW      = 0;
    st[i].failStreak = FAILS_TO_STALE;      // "lost" until first good reply
    st[i].pending    = false;
    st[i].reqId      = 0;
    st[i].sentAt     = 0;
    st[i].flashAt    = 0;
    st[i].lastPollAt = 0;
    st[i].mode       = 0;
    st[i].modeFails  = 0;
    st[i].step       = 0;
    for (int k = 0; k < 3; k++) {
      st[i].acc[k]    = NAN;
      st[i].accOk[k]  = false;
      st[i].oidEnc[k] = encOidBody(PDUS[i].oids[k]);
    }
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  int tick = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    drawBoot(tick++);
    delay(40);
  }
  lastWifiTry = millis();
}

// ================================================================= LOOP ===
void loop() {
  uint32_t now = millis();

  handleWiFi(now);
  handleButton(btnKey,  now, onKeyShort,  resetPeaks);
  handleButton(btnBoot, now, onBootShort, nullptr);

  if (WiFi.status() == WL_CONNECTED) {
    // each PDU runs on its own schedule, so a slow one never delays the other
    for (int i = 0; i < NUM_PDUS; i++) {
      if (!st[i].pending && (st[i].lastPollAt == 0 || now - st[i].lastPollAt >= POLL_MS)) {
        st[i].lastPollAt = now;
        startPoll(i, now);
      }
    }
    serviceUdp();
    checkTimeouts(now);
  }

  if (now - lastFrame >= 40) {      // ~25 fps for the animations
    lastFrame = now;
    drawFrame();
  }
}
