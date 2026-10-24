/*
  ============================================================================
   PDU//NETWATCH  -  CyberPower PDU power monitor for LilyGO T-Display-S3
                     (Arduino_GFX_Library edition)
  ============================================================================
   Polls a CyberPower PDU over SNMP (v2c, UDP/161) via WiFi and renders
   Watts / Volts / Hz plus a load-history graph in a neon cyberpunk style.

   Library needed:  "GFX Library for Arduino" (Arduino_GFX_Library) by
   moononournation. No config file editing required - pins are set below.

   Arduino IDE board settings (ESP32 core):
     Board ............ ESP32S3 Dev Module
     USB CDC On Boot .. Enabled
     Flash Size ....... 16MB
     PSRAM ............ OPI PSRAM
     Partition ........ 16M Flash (3MB APP/9.9MB FATFS)

   No SNMP library or font files are required.
  ============================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino_GFX_Library.h>
#include <vector>
#include <math.h>

// Custom types must be declared before any function. The Arduino IDE
// auto-generates function prototypes near the top of the sketch, and those
// prototypes need these types to already exist.
typedef std::vector<uint8_t> Bytes;
struct Reader { const uint8_t* p; size_t len; size_t pos; };
enum Align { A_LEFT, A_CENTER, A_RIGHT };

// ---------------------------------------------------------------- CONFIG ---
const char*     WIFI_SSID       = "YOUR_WIFI_SSID";
const char*     WIFI_PASS       = "YOUR_WIFI_PASSWORD";

IPAddress       PDU_IP(192, 168, 30, 21);
const char*     SNMP_COMMUNITY  = "public";
const uint16_t  SNMP_PORT       = 161;
const uint16_t  SNMP_LOCAL_PORT = 16161;
const int       SNMP_VERSION    = 1;      // 0 = SNMPv1, 1 = SNMPv2c (Home Assistant default)

const uint32_t  POLL_MS         = 2000;   // how often to query the PDU
const uint32_t  SNMP_TIMEOUT_MS = 1500;   // wait for a reply this long
const int       FAILS_TO_STALE  = 3;      // consecutive failures before "PDU LOST"

// OIDs taken from your Home Assistant config
const char* OIDS[3] = {
  "1.3.6.1.4.1.3808.1.1.3.2.3.1.1.7.1",   // Watts
  "1.3.6.1.4.1.3808.1.1.3.5.7.0",         // Volts (x10)
  "1.3.6.1.4.1.3808.1.1.3.5.8.0"          // Hz    (x10)
};
const float SCALE[3] = { 1.0f, 0.1f, 0.1f };

// Warning thresholds (North American 120V / 60Hz)
const float VOLT_MIN = 108.0f, VOLT_MAX = 132.0f;
const float HZ_MIN   = 59.5f,  HZ_MAX   = 60.5f;

// ------------------------------------------------------------- HARDWARE ---
// LilyGO T-Display-S3: ST7789 170x320, 8-bit parallel bus
#define PIN_POWER_ON  15   // must be HIGH for the LCD to run on battery
#define PIN_LCD_BL    38
#define PIN_LCD_RST    5
#define PIN_LCD_CS     6
#define PIN_LCD_DC     7
#define PIN_LCD_WR     8
#define PIN_LCD_RD     9

Arduino_DataBus *bus = new Arduino_ESP32PAR8Q(
  PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_WR, PIN_LCD_RD,
  39, 40, 41, 42, 45, 46, 47, 48);                 // D0..D7

Arduino_GFX *panel = new Arduino_ST7789(
  bus, PIN_LCD_RST, 1 /* rotation: landscape */, true /* IPS */,
  170, 320, 35, 0, 35, 0);

// Full-screen framebuffer for flicker-free drawing (lives in PSRAM)
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
#define C_SCAN     0x2947
#define C_BLACK    0x0000

// ---------------------------------------------------------------- STATE ---
WiFiUDP udp;

float watts = NAN, volts = NAN, hz = NAN, peakW = 0;
const int HIST_N = 77;                 // 76 segments * 4px = 304px graph
float hist[HIST_N];
int   histCount = 0;

int      failStreak   = FAILS_TO_STALE; // start "lost" until first good poll
uint32_t lastPoll     = 0;
uint32_t lastFrame    = 0;
uint32_t lastWifiTry  = 0;
uint32_t pollFlashAt  = 0;
bool     wasConnected = false;
int      glitchFrames = 0;
int      scanY        = 0;

Bytes oidEnc[3];                        // BER-encoded OID bodies

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

static Bytes buildGet(int32_t reqId) {
  Bytes varbinds;
  for (int i = 0; i < 3; i++) {
    Bytes vb = tlv(0x30, cat({ tlv(0x06, oidEnc[i]), Bytes{ 0x05, 0x00 } }));
    varbinds.insert(varbinds.end(), vb.begin(), vb.end());
  }
  Bytes community((const uint8_t*)SNMP_COMMUNITY,
                  (const uint8_t*)SNMP_COMMUNITY + strlen(SNMP_COMMUNITY));
  Bytes pdu = tlv(0xA0, cat({ encInt(reqId), encInt(0), encInt(0), tlv(0x30, varbinds) }));
  return tlv(0x30, cat({ encInt(SNMP_VERSION), tlv(0x04, community), pdu }));
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

static bool parseResponse(const uint8_t* buf, size_t n, int32_t expectId,
                          double out[3], bool ok[3]) {
  Reader r{ buf, n, 0 }; uint8_t t; size_t l;
  if (!readTL(r, t, l) || t != 0x30) return false;              // message
  if (!readTL(r, t, l) || t != 0x02) return false; r.pos += l;  // version
  if (!readTL(r, t, l) || t != 0x04) return false; r.pos += l;  // community
  if (!readTL(r, t, l) || t != 0xA2) return false;              // GetResponse
  if (!readTL(r, t, l) || t != 0x02) return false;
  int32_t id = (int32_t)decSigned(r.p + r.pos, l); r.pos += l;
  if (id != expectId) return false;
  if (!readTL(r, t, l) || t != 0x02) return false;
  int32_t err = (int32_t)decSigned(r.p + r.pos, l); r.pos += l;
  if (!readTL(r, t, l) || t != 0x02) return false; r.pos += l;  // error-index
  if (!readTL(r, t, l) || t != 0x30) return false;
  size_t listEnd = r.pos + l;

  while (r.pos < listEnd) {
    if (!readTL(r, t, l) || t != 0x30) return false;
    size_t vbEnd = r.pos + l;
    if (!readTL(r, t, l) || t != 0x06) return false;
    const uint8_t* oid = r.p + r.pos; size_t oidLen = l; r.pos += l;
    if (!readTL(r, t, l)) return false;
    const uint8_t* val = r.p + r.pos; size_t valLen = l;

    for (int i = 0; i < 3; i++) {
      if (oidEnc[i].size() == oidLen && memcmp(oidEnc[i].data(), oid, oidLen) == 0) {
        double v;
        if (decodeValue(t, val, valLen, v)) { out[i] = v; ok[i] = true; }
      }
    }
    r.pos = vbEnd;
  }
  if (err != 0) Serial.printf("[SNMP] error-status %d\n", err);
  return err == 0 || ok[0] || ok[1] || ok[2];
}

static bool snmpPoll(double out[3], bool ok[3]) {
  static int32_t reqId = 1000;
  if (++reqId > 0x7FFFFFF0) reqId = 1000;

  while (udp.parsePacket() > 0) udp.flush();     // drop stale replies

  Bytes msg = buildGet(reqId);
  udp.beginPacket(PDU_IP, SNMP_PORT);
  udp.write(msg.data(), msg.size());
  udp.endPacket();

  uint32_t start = millis();
  uint8_t buf[512];
  while (millis() - start < SNMP_TIMEOUT_MS) {
    int sz = udp.parsePacket();
    if (sz > 0) {
      int n = udp.read(buf, sizeof(buf));
      if (n > 0 && parseResponse(buf, n, reqId, out, ok)) return true;
    }
    delay(5);
  }
  return false;
}

// ============================================================== POLLING ===
static void pushHistory(float w) {
  if (histCount < HIST_N) {
    hist[histCount++] = w;
  } else {
    memmove(hist, hist + 1, sizeof(float) * (HIST_N - 1));
    hist[HIST_N - 1] = w;
  }
}

static void doPoll() {
  double v[3] = { NAN, NAN, NAN };
  bool ok[3]  = { false, false, false };

  if (snmpPoll(v, ok)) {
    failStreak  = 0;
    pollFlashAt = millis();
    if (ok[0]) { watts = v[0] * SCALE[0]; pushHistory(watts); if (watts > peakW) peakW = watts; }
    if (ok[1]) volts = v[1] * SCALE[1];
    if (ok[2]) hz    = v[2] * SCALE[2];
    Serial.printf("[PDU] %.0f W  %.1f V  %.1f Hz\n", watts, volts, hz);
  } else {
    failStreak++;
    Serial.printf("[PDU] no reply (%d)\n", failStreak);
    if (failStreak >= FAILS_TO_STALE) { watts = volts = hz = NAN; }
  }
}

// ================================================================= WIFI ===
static void handleWiFi(uint32_t now) {
  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wasConnected) {
    Serial.printf("[WiFi] connected, IP %s\n", WiFi.localIP().toString().c_str());
    udp.stop();
    udp.begin(SNMP_LOCAL_PORT);
    lastPoll = 0;
  }
  if (!connected && now - lastWifiTry > 10000) {
    lastWifiTry = now;
    Serial.println("[WiFi] reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  wasConnected = connected;
}

// ======================================================= DRAW HELPERS ===
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
  const int g  = 1;                       // gap between segments
  const int hh = h / 2;
  const int hw = w - 2 * t - 2 * g;       // horizontal segment length
  const int uy = y + t + g,          uh = hh - t / 2 - t - 2 * g;      // upper vertical
  const int ly = y + hh + t / 2 + g, lh = (y + h - t) - (y + hh + t / 2) - 2 * g;

  if (m & 0x01) gfx->fillRect(x + t + g, y,              hw, t,  c);   // a
  if (m & 0x02) gfx->fillRect(x + w - t, uy,             t,  uh, c);   // b
  if (m & 0x04) gfx->fillRect(x + w - t, ly,             t,  lh, c);   // c
  if (m & 0x08) gfx->fillRect(x + t + g, y + h - t,      hw, t,  c);   // d
  if (m & 0x10) gfx->fillRect(x,         ly,             t,  lh, c);   // e
  if (m & 0x20) gfx->fillRect(x,         uy,             t,  uh, c);   // f
  if (m & 0x40) gfx->fillRect(x + t + g, y + hh - t / 2, hw, t,  c);   // g
}

// Draws a numeric string right-aligned at rightX. Supports 0-9, '-', '.', ' '.
// Unlit segments are drawn in C_GHOST; a shadow copy is offset by shadowDx.
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

// ======================================================== SCREEN PARTS ===
static void drawBackground() {
  gfx->fillScreen(C_BG);
  for (int x = 0; x < 320; x += 16) gfx->drawFastVLine(x, 20, 150, C_GRID);
  for (int y = 20; y < 170; y += 16) gfx->drawFastHLine(0, y, 320, C_GRID);
}

static void drawHeader(bool pduOnline) {
  // slanted magenta title bar
  gfx->fillRect(0, 0, 200, 18, C_MAGENTA);
  gfx->fillTriangle(200, 0, 214, 0, 200, 18, C_MAGENTA);
  gfx->drawFastHLine(214, 18, 106, C_CYAN);
  text("CYBERPOWER//NETWATCH", 6, 5, 1, C_BLACK);
  text("CYBERPOWER//NETWATCH", 7, 5, 1, C_BLACK);   // faux bold

  // status text
  const char* status; uint16_t sc;
  if (WiFi.status() != WL_CONNECTED) { status = "NO WIFI";  sc = C_RED;  }
  else if (!pduOnline)               { status = "PDU LOST"; sc = C_RED;  }
  else                               { status = "ONLINE";   sc = C_CYAN; }
  bool blink = (sc == C_RED) && ((millis() / 400) % 2);
  text(status, 292, 5, 1, blink ? C_DIMMAG : sc, A_RIGHT);

  // poll activity pip
  if (millis() - pollFlashAt < 200) gfx->fillRect(220, 6, 5, 5, C_YELLOW);
  else                              gfx->drawRect(220, 6, 5, 5, C_DIMCYAN);

  // WiFi signal bars
  int bars = 0;
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

static void drawWattsPanel(bool glitch) {
  cutPanel(2, 24, 200, 78, C_CYAN, 10);
  text("POWER DRAW // LOAD", 10, 29, 1, C_YELLOW);

  // decorative barcode
  for (int i = 0; i < 11; i++) gfx->fillRect(150 + i * 4, 28, (i * 7 % 3) + 1, 7, C_DIMMAG);

  if (isnan(watts)) {
    text("NO DATA", 102 + (glitch ? random(-3, 4) : 0), 52, 3, C_RED, A_CENTER);
  } else {
    char buf[12];
    float w = watts > 9999 ? 9999 : watts;
    snprintf(buf, sizeof(buf), "%4.0f", w);       // padded -> ghost "8888"
    int jx = glitch ? random(-4, 5) : 0;
    sevenSeg(buf, 166, 40, 28, 48, 6, 6, C_CYAN, C_MAGENTA, -3 + jx);
    text("W", 172, 64, 3, C_MAGENTA);
  }

  char pk[24];
  snprintf(pk, sizeof(pk), "PEAK %.0fW", peakW);
  text(pk, 10, 92, 1, C_DIMCYAN);
}

static void drawStatPanel(int x, int y, int w, int h, const char* label,
                          float v, const char* unit, bool warn) {
  cutPanel(x, y, w, h, warn ? C_RED : C_MAGENTA, 6);
  text(label, x + 6, y + 4, 1, C_YELLOW);

  char buf[12];
  if (isnan(v)) strcpy(buf, " --.-");
  else          snprintf(buf, sizeof(buf), "%5.1f", v);

  sevenSeg(buf, x + w - 20, y + 13, 12, 22, 3, 3,
           warn ? C_RED : C_CYAN, C_DIMMAG, -1);
  text(unit, x + w - 4, y + h - 11, 1, C_MAGENTA, A_RIGHT);
}

static void drawGraph() {
  const int px = 2, py = 106, pw = 316, ph = 62;
  const int gx = 8, gy = 118, gw = 304, gh = 46;
  cutPanel(px, py, pw, ph, C_CYAN, 8);

  float maxV = 50;
  for (int i = 0; i < histCount; i++) if (hist[i] > maxV) maxV = hist[i];
  float scale = ceilf(maxV * 1.25f / 50.0f) * 50.0f;

  text("LOAD HISTORY", 12, 109, 1, C_YELLOW);
  char sbuf[16];
  snprintf(sbuf, sizeof(sbuf), "MAX %.0fW", scale);
  text(sbuf, 310, 109, 1, C_DIMCYAN, A_RIGHT);

  for (int i = 1; i < 4; i++) {
    int yy = gy + gh * i / 4;
    for (int x = gx; x < gx + gw; x += 4) gfx->drawPixel(x, yy, C_DIMMAG);
  }

  if (histCount < 2) return;
  const int step = gw / (HIST_N - 1);
  const int bot  = gy + gh;
  auto ptY = [&](float w) { return bot - (int)((w / scale) * gh); };

  for (int i = 1; i < histCount; i++) {
    int x0 = gx + gw - (histCount - i) * step, x1 = x0 + step;
    int y0 = ptY(hist[i - 1]), y1 = ptY(hist[i]);
    gfx->fillTriangle(x0, y0, x1, y1, x0, bot, C_DIMCYAN);
    gfx->fillTriangle(x1, y1, x1, bot, x0, bot, C_DIMCYAN);
  }
  for (int i = 1; i < histCount; i++) {
    int x0 = gx + gw - (histCount - i) * step, x1 = x0 + step;
    int y0 = ptY(hist[i - 1]), y1 = ptY(hist[i]);
    gfx->drawLine(x0, y0, x1, y1, C_CYAN);
    gfx->drawLine(x0, y0 - 1, x1, y1 - 1, C_CYAN);
  }
  gfx->fillCircle(gx + gw, ptY(hist[histCount - 1]), 2, C_MAGENTA);
}

static void drawGlitchSlices() {
  for (int i = 0; i < 3; i++) {
    int y = random(20, 168), x = random(0, 280), w = random(10, 60);
    gfx->fillRect(x, y, w, 2, random(2) ? C_MAGENTA : C_CYAN);
  }
}

static void drawFrame() {
  bool pduOnline = failStreak < FAILS_TO_STALE;

  if (glitchFrames > 0) glitchFrames--;
  else if (random(100) < 2) glitchFrames = random(2, 5);
  bool glitch = glitchFrames > 0;

  drawBackground();
  drawHeader(pduOnline);
  drawWattsPanel(glitch);
  drawStatPanel(208, 24, 110, 38, "VOLTS", volts, "V",
                !isnan(volts) && (volts < VOLT_MIN || volts > VOLT_MAX));
  drawStatPanel(208, 64, 110, 38, "FREQ",  hz,    "Hz",
                !isnan(hz) && (hz < HZ_MIN || hz > HZ_MAX));
  drawGraph();

  // moving CRT scan line
  scanY = (scanY + 3) % 170;
  gfx->drawFastHLine(0, scanY, 320, C_SCAN);

  if (glitch) drawGlitchSlices();
  gfx->flush();                     // push framebuffer to the panel
}

static void drawBoot(int tick) {
  gfx->fillScreen(C_BG);
  for (int y = 0; y < 170; y += 3) gfx->drawFastHLine(0, y, 320, C_GRID);

  int jx = (tick % 9 == 0) ? random(-3, 4) : 0;
  text("JACKING IN", 160 + jx - 2, 48, 3, C_MAGENTA, A_CENTER);
  text("JACKING IN", 160,           47, 3, C_CYAN,    A_CENTER);

  char buf[48];
  snprintf(buf, sizeof(buf), "NET: %s", WIFI_SSID);
  text(buf, 160, 86, 2, C_YELLOW, A_CENTER);

  int w = (tick * 6) % 238;
  gfx->drawRect(40, 112, 240, 10, C_DIMCYAN);
  gfx->fillRect(41, 113, w, 8, C_MAGENTA);

  text("PDU//NETWATCH  v1.0", 160, 142, 1, C_DIMCYAN, A_CENTER);
  gfx->flush();
}

// ================================================================ SETUP ===
void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  Serial.begin(115200);
  randomSeed(esp_random());

  if (!gfx->begin()) {
    Serial.println("[GFX] begin() failed - framebuffer allocation? Enable OPI PSRAM.");
  }
  gfx->setTextWrap(false);
  gfx->fillScreen(C_BLACK);
  gfx->flush();

  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);   // backlight on after the first clear

  for (int i = 0; i < 3; i++) oidEnc[i] = encOidBody(OIDS[i]);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  int tick = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    drawBoot(tick++);
    delay(50);
  }
  lastWifiTry = millis();
}

// ================================================================= LOOP ===
void loop() {
  uint32_t now = millis();
  handleWiFi(now);

  if (WiFi.status() == WL_CONNECTED && (lastPoll == 0 || now - lastPoll >= POLL_MS)) {
    lastPoll = now;
    doPoll();
  }

  if (now - lastFrame >= 40) {      // ~25 fps for the animations
    lastFrame = now;
    drawFrame();
  }
}
