// Kettler-Temp v3 : ESP32-S3 + DS18B20 (GPIO 4)
//  * يرسل حرارة المحرك للتطبيق عبر Nordic UART  "Temp: 26.44 C"
//  * وسيط FTMS: يظهر لتطبيقات مثل Kinomap وZwift كمشاية، ويمرّر أوامرها إلى المشاية الحقيقية
//    ولا يتصل بالمشاية إلا عند الحاجة (تطبيق متصل به، أو ارتفاع الحرارة، أو زر الربط)
//  * حماية: عند تجاوز حد الخطر يخفّض السرعة تدريجيًا ثم يوقف المشاية ويمنع إعادة التشغيل
//    حتى تنزل الحرارة تحت حد التحذير. حدّا الحرارة (W و H) يرسلهما التطبيق ويحفظهما.
//  * تحليل صحة المحرك (MotorHealth.h) وتحديث البرنامج بالبلوتوث (OTA، الرمز OTA_PIN)
// المكتبة: NimBLE-Arduino 1.4.x
#include <NimBLEDevice.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <Update.h>
#include "MotorHealth.h"

#define PIN_DS   4
#define NUS_SVC  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define DEV_NAME "Kettler-Temp"
#define OTA_SVC  "6e400010-b5a3-f393-e0a9-e50e24dcca9e"
#define OTA_CTL  "6e400011-b5a3-f393-e0a9-e50e24dcca9e"
#define OTA_DAT  "6e400012-b5a3-f393-e0a9-e50e24dcca9e"
#define OTA_PIN  "4827"      // رمز تحديث البرنامج: غيّره هنا وفي حقل الرمز بالتطبيق

#define U16(x) NimBLEUUID((uint16_t)(x))

// الأنواع هنا في الأعلى: مترجم Arduino يولّد تعريفات الدوال تلقائيًا قبل أول دالة
struct Fwd { uint8_t n; uint8_t d[10]; uint32_t ms; };
struct LockState { uint32_t last; bool stopped; float lastTarget; };

OneWire ow(PIN_DS);
DallasTemperature ds(&ow);
NimBLEServer* gServer = nullptr;
NimBLECharacteristic* txChar = nullptr;
volatile bool connected = false;     // تطبيق الويب اشترك في قناة الحرارة
volatile bool needStatus = false;
volatile uint32_t connMs = 0;

Preferences mhPrefs;
Preferences limPrefs;
Preferences fmPrefs;
mh::Engine mhe;

// ---------- حدود الحرارة ----------
float warnC = 60.0f, hotC = 80.0f;
volatile float pendW = 0, pendH = 0;
volatile bool lockOn = false;        // حماية الحرارة فعّالة
volatile bool hardStop = false;      // الحرارة فوق الخطر بفارق كبير: إيقاف فوري
volatile bool manualLink = false;    // زر "ربط المشاية" في التطبيق
volatile uint32_t dOffAt = 0;        // لحظة ضغط "فصل": تُلغي أسباب الاتصال الضعيفة بعدها
volatile float lastTemp = -100.0f;
uint8_t hotCnt = 0;
float effWarn() { return warnC < hotC ? warnC : hotC - 5.0f; }

// ---------- وحدة صحة المحرك ----------
bool mhLoad(void* p, size_t n)       { return mhPrefs.getBytes("s", p, n) == n; }
void mhSave(const void* p, size_t n) { mhPrefs.putBytes("s", p, n); }
void mhSleep(uint32_t ms)            { delay(ms); }
void mhTx(const char* d, size_t n) {
  if (!connected || !txChar) return;
  txChar->setValue((const uint8_t*)d, n);
  txChar->notify();
}
void sendLine(const char* s) { mhTx(s, strlen(s)); }

// ---------- استقبال أوامر تطبيق الويب ----------
char lineBuf[48];
size_t lineLen = 0;
void handleLine(const char* l) {
  if (l[0] == 'M') { mhe.pushCommand(l); return; }
  if (l[0] == 'W') { float v = atof(l + 1); if (v >= 20 && v <= 120) pendW = v; return; }
  if (l[0] == 'H') { float v = atof(l + 1); if (v >= 20 && v <= 120) pendH = v; return; }
  if (l[0] == 'C') { manualLink = true; dOffAt = 0; return; }
  if (l[0] == 'D') { manualLink = false; dOffAt = millis() | 1; return; }
}
void feedByte(char c) {
  if (c == '\n' || c == '\r') {
    if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
  } else if (lineLen < sizeof(lineBuf) - 1) {
    lineBuf[lineLen++] = c;
  } else {
    lineLen = 0;
  }
}
class RxCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) {
    auto v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) feedByte((char)v[i]);
  }
};
class TxCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, ble_gap_conn_desc*, uint16_t sub) {
    connected = (sub != 0);
    if (connected) { connMs = millis(); needStatus = true; lineLen = 0; }
  }
};
class SrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) { if (s->getConnectedCount() < 2) NimBLEDevice::startAdvertising(); }
  void onDisconnect(NimBLEServer*) { NimBLEDevice::startAdvertising(); }
};

// ---------- تحديث البرنامج بالبلوتوث (OTA) ----------
// الأوامر (عبر OTA_CTL): B<حجم>,<crc32 hex>,<رمز>  بدء | E إنهاء | X إلغاء
// الردود: OK | A:<عدد البايتات المكتوبة> | DONE | ERR:<سبب>
// البيانات (عبر OTA_DAT) تدخل مخزنًا حلقيًا وتُكتب في الذاكرة من الحلقة الرئيسية
static const uint32_t OTA_RING = 16384;           // قوة 2
static uint8_t otaRing[OTA_RING];
volatile uint32_t ringHead = 0, ringTail = 0;
volatile bool otaOverflow = false;
NimBLECharacteristic* otaCtl = nullptr;
char otaCmd[48];
volatile bool otaCmdReady = false;
bool otaActive = false;
uint32_t otaSize = 0, otaWritten = 0, otaLastAck = 0, otaCrcExp = 0, otaCrcRun = 0, otaLastMs = 0, restartAt = 0;

uint32_t crcUpdate(uint32_t crc, const uint8_t* d, size_t n) {
  crc = ~crc;
  while (n--) {
    crc ^= *d++;
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
void otaReply(const char* s) {
  if (!connected || !otaCtl) return;
  otaCtl->setValue((const uint8_t*)s, strlen(s));
  otaCtl->notify();
}
void otaAbort(const char* msg) {
  if (otaActive) Update.abort();
  otaActive = false;
  ringTail = ringHead;
  if (msg) otaReply(msg);
}
void ringPush(const uint8_t* d, size_t n) {
  if ((uint32_t)(ringHead - ringTail) + n > OTA_RING) { otaOverflow = true; return; }
  for (size_t i = 0; i < n; i++) otaRing[(ringHead + i) & (OTA_RING - 1)] = d[i];
  __sync_synchronize();
  ringHead += n;
}
class OtaCtlCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) {
    auto v = c->getValue();
    if (otaCmdReady) return;
    size_t n = v.length() < sizeof(otaCmd) - 1 ? v.length() : sizeof(otaCmd) - 1;
    for (size_t i = 0; i < n; i++) otaCmd[i] = (char)v[i];
    otaCmd[n] = 0;
    otaCmdReady = true;
  }
};
class OtaDatCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) {
    auto v = c->getValue();
    if (otaActive) ringPush((const uint8_t*)v.data(), v.length());
  }
};

void otaCommand(const char* c) {
  if (c[0] == 'X') { otaAbort("ERR:cancel"); return; }
  if (c[0] == 'B') {
    if (otaActive) otaAbort(nullptr);
    char pin[16] = {0};
    unsigned long sz = 0, crc = 0;
    if (sscanf(c + 1, "%lu,%lx,%15s", &sz, &crc, pin) != 3) { otaReply("ERR:format"); return; }
    if (strcmp(pin, OTA_PIN) != 0) { otaReply("ERR:pin"); return; }
    if (sz < 50000UL || sz > 1300000UL) { otaReply("ERR:size"); return; }
    if (!Update.begin(sz)) { otaReply("ERR:begin"); return; }
    otaActive = true; otaSize = sz; otaCrcExp = (uint32_t)crc;
    otaWritten = 0; otaLastAck = 0; otaCrcRun = 0; otaOverflow = false;
    ringTail = ringHead; otaLastMs = millis();
    otaReply("OK");
    return;
  }
  if (c[0] == 'E') {
    if (!otaActive || otaWritten != otaSize) { otaAbort("ERR:incomplete"); return; }
    if (otaCrcRun != otaCrcExp) { otaAbort("ERR:crc"); return; }
    bool ok = Update.end(true);
    otaActive = false;
    if (!ok) { otaReply("ERR:end"); return; }
    otaReply("DONE");
    restartAt = millis() + 1500;
    if (restartAt == 0) restartAt = 1;
  }
}

void otaPump() {
  if (otaCmdReady) {
    char tmp[48];
    memcpy(tmp, (const void*)otaCmd, sizeof tmp);
    tmp[sizeof tmp - 1] = 0;
    otaCmdReady = false;
    otaCommand(tmp);
  }
  if (restartAt && (int32_t)(millis() - restartAt) >= 0) ESP.restart();
  if (!otaActive) return;
  if (!connected) { otaAbort(nullptr); return; }
  if (otaOverflow) { otaAbort("ERR:overflow"); return; }
  if ((uint32_t)(millis() - otaLastMs) > 20000) { otaAbort("ERR:timeout"); return; }

  int rounds = 0;
  while (ringTail != ringHead && rounds < 6) {
    uint32_t avail = ringHead - ringTail;
    if (otaWritten == 0 && avail < 36) break;           // نحتاج بداية الملف للتحقق منه
    size_t n = avail > 512 ? 512 : avail;
    uint8_t b[512];
    for (size_t i = 0; i < n; i++) b[i] = otaRing[(ringTail + i) & (OTA_RING - 1)];
    if (otaWritten == 0) {                              // علامة برنامج ESP32 عند البايت 32
      if (b[0] != 0xE9 || b[32] != 0x32 || b[33] != 0x54 || b[34] != 0xCD || b[35] != 0xAB) { otaAbort("ERR:notapp"); return; }
    }
    if (otaWritten + n > otaSize) { otaAbort("ERR:over"); return; }
    if (Update.write(b, n) != n) { otaAbort("ERR:write"); return; }
    otaCrcRun = crcUpdate(otaCrcRun, b, n);
    otaWritten += n;
    ringTail += n;
    otaLastMs = millis();
    rounds++;
  }
  if (otaWritten - otaLastAck >= 2048 || (otaWritten == otaSize && otaLastAck != otaWritten)) {
    otaLastAck = otaWritten;
    char m[24];
    snprintf(m, sizeof m, "A:%lu", (unsigned long)otaWritten);
    otaReply(m);
  }
}

// ---------- وسيط FTMS: الجهة التي تتصل بها Kinomap وZwift ----------
NimBLECharacteristic *fmFeat = nullptr, *fmData = nullptr, *fmStatus = nullptr,
                     *fmSpd = nullptr, *fmInc = nullptr, *fmCp = nullptr;
volatile uint32_t ftmsLastAct = 0;   // آخر نشاط من تطبيق متصل بالوسيط
uint8_t fmFeatB[8] = {0x0C, 0x10, 0, 0, 0x03, 0, 0, 0};          // مسافة+ميل+زمن | هدف السرعة والميل
uint8_t fmSpdB[6]  = {0x64, 0x00, 0xD0, 0x07, 0x32, 0x00};       // 1 إلى 20 كم/س، خطوة 0.5
uint8_t fmIncB[6]  = {0x00, 0x00, 0x96, 0x00, 0x0A, 0x00};       // 0 إلى 15%، خطوة 1
volatile bool fmGot = false;         // قرأت اللوحة مواصفات المشاية الحقيقية

// نتيجة أمر التحكم: 1 نجاح، 2 غير مدعوم، 5 غير مسموح (الحماية فعّالة)
uint8_t cpDecide(uint8_t op, bool lock, bool* fwd) {
  *fwd = false;
  switch (op) {
    case 0x00: case 0x01: return 1;
    case 0x02: case 0x03: case 0x07: if (lock) return 5; *fwd = true; return 1;
    case 0x08: *fwd = true; return 1;
    default: return 2;
  }
}

uint8_t cpRep[4][3];
volatile uint8_t cpRH = 0, cpRT = 0;
void cpReplyPush(uint8_t op, uint8_t res) {
  uint8_t nx = (cpRH + 1) & 3;
  if (nx == cpRT) return;
  cpRep[cpRH][0] = 0x80; cpRep[cpRH][1] = op; cpRep[cpRH][2] = res;
  __sync_synchronize();
  cpRH = nx;
}
Fwd fwdQ[16];
volatile uint8_t fwdH = 0, fwdT = 0;
void fwdPush(const uint8_t* d, size_t n) {
  if (n > 10) n = 10;
  uint8_t nx = (fwdH + 1) & 15;
  if (nx == fwdT) return;
  fwdQ[fwdH].n = n; memcpy(fwdQ[fwdH].d, d, n); fwdQ[fwdH].ms = millis();
  __sync_synchronize();
  fwdH = nx;
}
class FmCpCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) {
    auto v = c->getValue();
    size_t n = v.length();
    if (!n) return;
    uint8_t b[10];
    if (n > sizeof b) n = sizeof b;
    for (size_t i = 0; i < n; i++) b[i] = (uint8_t)v[i];
    ftmsLastAct = millis();
    bool fwd;
    uint8_t res = cpDecide(b[0], lockOn, &fwd);
    cpReplyPush(b[0], res);
    if (fwd) fwdPush(b, n);
  }
  void onSubscribe(NimBLECharacteristic*, ble_gap_conn_desc*, uint16_t) { ftmsLastAct = millis(); }
};
class FmSubCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, ble_gap_conn_desc*, uint16_t) { ftmsLastAct = millis(); }
};

// ---------- الاتصال بالمشاية الحقيقية (مهمة منفصلة) ----------
// 0 غير متصلة، 1 بحث، 2 متصلة، 3 فشل
volatile uint8_t guardState = 0;
volatile bool guardWant = false;
volatile bool tmLinked = false;
volatile bool needCtrl = true;
NimBLEClient* tmClient = nullptr;
NimBLERemoteCharacteristic* tmCp = nullptr;
NimBLEAddress* tmAddr = nullptr;
volatile bool tmFound = false;
static const uint8_t CMD_REQ[] = {0x00};          // Request Control

uint8_t tmPkt[32]; volatile uint8_t tmPktLen = 0; volatile bool tmPktNew = false;
uint8_t tmSt[16];  volatile uint8_t tmStLen = 0;  volatile bool tmStNew = false;
volatile float tmSpeed = 0, tmIncl = 0;
volatile uint32_t tmSpeedMs = 0;

void tmDataNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n < 2 || n > sizeof tmPkt) return;
  memcpy(tmPkt, d, n); tmPktLen = (uint8_t)n; tmPktNew = true;
  uint16_t f = d[0] | (d[1] << 8);
  size_t o = 2;
  if (!(f & 1)) { if (n >= o + 2) { tmSpeed = (d[o] | (d[o + 1] << 8)) * 0.01f; tmSpeedMs = millis(); } o += 2; }
  if (f & 2) o += 2;
  if (f & 4) o += 3;
  if (f & 8) { if (n >= o + 2) tmIncl = (int16_t)(d[o] | (d[o + 1] << 8)) * 0.1f; }
}
void tmStatusNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n < 1 || n > sizeof tmSt) return;
  memcpy(tmSt, d, n); tmStLen = (uint8_t)n; tmStNew = true;
}
void cpNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n >= 3 && d[0] == 0x80 && d[2] == 0x05) needCtrl = true;   // 5 = لا صلاحية تحكم
}

class ScanCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* d) {
    bool ok = d->isAdvertisingService(U16(0x1826));
    if (!ok) {
      std::string nm = d->getName();
      for (size_t i = 0; i < nm.size(); i++) nm[i] = toupper((unsigned char)nm[i]);
      if (nm.find("KETTLER") != std::string::npos && nm.find("TEMP") == std::string::npos) ok = true;
    }
    if (ok) {
      delete tmAddr;
      tmAddr = new NimBLEAddress(d->getAddress());
      tmFound = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};
ScanCb scanCb;

bool readInto(NimBLERemoteService* s, uint16_t uuid, uint8_t* dst, size_t n) {
  NimBLERemoteCharacteristic* c = s->getCharacteristic(U16(uuid));
  if (!c || !c->canRead()) return false;
  auto v = c->readValue();
  if (v.size() < n) return false;
  memcpy(dst, (const uint8_t*)v.data(), n);
  return true;
}

bool tmConnect() {
  guardState = 1;
  tmFound = false;
  NimBLEScan* sc = NimBLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(&scanCb, false);
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(80);
  sc->start(6, false);
  sc->clearResults();
  if (!tmFound || !tmAddr) return false;
  if (!tmClient) { tmClient = NimBLEDevice::createClient(); tmClient->setConnectTimeout(8); }
  if (!tmClient->connect(*tmAddr)) return false;
  NimBLERemoteService* s = tmClient->getService(U16(0x1826));
  if (!s) { tmClient->disconnect(); return false; }
  tmCp = s->getCharacteristic(U16(0x2AD9));
  if (!tmCp) { tmClient->disconnect(); return false; }
  tmCp->subscribe(false, cpNotify);                       // ردود التحكم (indication)
  NimBLERemoteCharacteristic* c = s->getCharacteristic(U16(0x2ACD));
  if (c && c->canNotify()) c->subscribe(true, tmDataNotify);
  c = s->getCharacteristic(U16(0x2ADA));
  if (c && c->canNotify()) c->subscribe(true, tmStatusNotify);
  if (!fmGot) {                                           // مواصفات المشاية مرة واحدة
    uint8_t f[8], sp[6], ic[6];
    bool a = readInto(s, 0x2ACC, f, 8), b = readInto(s, 0x2AD4, sp, 6), d = readInto(s, 0x2AD5, ic, 6);
    if (a) memcpy(fmFeatB, f, 8);
    if (b) memcpy(fmSpdB, sp, 6);
    if (d) memcpy(fmIncB, ic, 6);
    if (a || b || d) fmGot = true;
  }
  needCtrl = true;
  return true;
}

void ensureCtrl() {
  if (needCtrl) {
    tmCp->writeValue(CMD_REQ, 1, false);
    needCtrl = false;
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}
void flushFwd() {
  int n = 0;
  while (fwdT != fwdH && n++ < 8) {
    Fwd f = fwdQ[fwdT];
    fwdT = (fwdT + 1) & 15;
    if ((uint32_t)(millis() - f.ms) < 15000) {
      ensureCtrl();
      tmCp->writeValue(f.d, f.n, false);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

// قرار الحماية: تخفيض السرعة 1 كم/س كل 1.5 ثانية حتى 3 كم/س ثم إيقاف، وإيقاف فوري عند الخطر الشديد
size_t lockDecide(LockState& s, uint32_t now, bool fresh, float cur, bool hard, uint8_t out[3]) {
  uint32_t gap = s.stopped ? 3000 : 1500;
  if ((uint32_t)(now - s.last) < gap) return 0;
  s.last = now;
  if (hard || s.stopped || !fresh || cur <= 3.5f) {
    s.stopped = true; out[0] = 0x08; out[1] = 0x01; return 2;
  }
  float base = cur;
  if (s.lastTarget > 0 && s.lastTarget < base) base = s.lastTarget;
  float tgt = base - 1.0f;
  if (tgt < 3.0f) tgt = 3.0f;
  s.lastTarget = tgt;
  uint16_t v = (uint16_t)(tgt * 100.0f + 0.5f);
  out[0] = 0x02; out[1] = v & 255; out[2] = v >> 8;
  return 3;
}

// متى يجب أن تكون اللوحة متصلة بالمشاية؟
// الحماية تعمل دائمًا. زر "ربط المشاية" يعمل ما دام تطبيق ما متصلًا باللوحة (يسقط بعد 20 ثانية من غياب كل التطبيقات).
// تطبيق مثل Kinomap/Zwift يُبقي الاتصال ما دام أرسل أمرًا خلال آخر 20 ثانية وبعد آخر ضغط على "فصل".
// وتستعد اللوحة أيضًا عندما تقترب الحرارة من حد التحذير (بعد دقيقة من الفصل).
#define IDLE_MS 20000UL
bool wantLink(uint32_t now, bool lock, bool manual, uint32_t ftmsAct, uint32_t dOff, float temp, float warn, bool gone) {
  if (lock) return true;
  if (manual && !gone) return true;
  bool fresh = ftmsAct && (uint32_t)(now - ftmsAct) < IDLE_MS;
  if (fresh && dOff && (int32_t)(ftmsAct - dOff) <= 0) fresh = false;
  if (gone) fresh = false;
  bool weakOk = !dOff || (uint32_t)(now - dOff) > 60000UL;
  bool standby = weakOk && temp > -50.0f && temp >= warn - 2.0f;
  return fresh || standby;
}

void guardTask(void*) {
  LockState ls = {0, false, 0};
  uint32_t linkAt = 0;
  for (;;) {
    bool want = guardWant;
    bool conn = tmClient && tmClient->isConnected();
    tmLinked = conn;
    if (want && !conn) {
      if (tmConnect()) { guardState = 2; linkAt = millis(); ls = {0, false, 0}; tmLinked = true; }
      else { guardState = 3; vTaskDelay(pdMS_TO_TICKS(3000)); }
    } else if (want && conn) {
      guardState = 2;
      if (lockOn) {
        uint32_t now = millis();
        bool fresh = (uint32_t)(now - tmSpeedMs) < 4000;
        if (fresh || (uint32_t)(now - linkAt) >= 2500) {
          uint8_t cmd[3];
          size_t n = lockDecide(ls, now, fresh, tmSpeed, hardStop, cmd);
          if (n) { ensureCtrl(); tmCp->writeValue(cmd, n, false); }
        }
        fwdT = fwdH;                                  // أوامر التطبيقات مرفوضة أثناء الحماية
      } else {
        ls = {0, false, 0};
        flushFwd();
      }
    } else if (!want && conn) {
      tmClient->disconnect();
      guardState = 0; tmLinked = false;
    } else {
      guardState = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void bridgePump(uint32_t now) {
  while (cpRT != cpRH) {                              // ردود أوامر Kinomap/Zwift
    fmCp->setValue(cpRep[cpRT], 3);
    fmCp->indicate();
    cpRT = (cpRT + 1) & 3;
  }
  if (tmPktNew) {                                     // بيانات المشاية إلى التطبيقات
    tmPktNew = false;
    uint8_t b[32]; uint8_t n = tmPktLen;
    memcpy(b, (const void*)tmPkt, n);
    fmData->setValue(b, n);
    fmData->notify();
    if ((uint32_t)(now - tmSpeedMs) < 4000) mhe.feedSpeed(now, tmSpeed, tmIncl);
  } else if (!tmLinked && ftmsLastAct && (uint32_t)(now - ftmsLastAct) < 60000) {
    static uint32_t lastZero = 0;                     // قبل اكتمال الاتصال: آخر قراءة معروفة (أو صفر)
    if ((uint32_t)(now - lastZero) >= 1000) {
      static const uint8_t z[4] = {0, 0, 0, 0};
      if (tmPktLen > 0 && (uint32_t)(now - tmSpeedMs) < 60000UL) {
        uint8_t b[32]; uint8_t n = tmPktLen;
        memcpy(b, (const void*)tmPkt, n);
        fmData->setValue(b, n);
      } else {
        fmData->setValue(z, 4);
      }
      fmData->notify();
      lastZero = now;
    }
  }
  if (tmStNew) {
    tmStNew = false;
    uint8_t b[16]; uint8_t n = tmStLen;
    memcpy(b, (const void*)tmSt, n);
    fmStatus->setValue(b, n);
    fmStatus->notify();
  }
  if (fmGot) {                                        // حفظ مواصفات المشاية وإظهارها للتطبيقات
    fmGot = false;
    fmFeat->setValue(fmFeatB, 8); fmSpd->setValue(fmSpdB, 6); fmInc->setValue(fmIncB, 6);
    uint8_t blob[20];
    memcpy(blob, fmFeatB, 8); memcpy(blob + 8, fmSpdB, 6); memcpy(blob + 14, fmIncB, 6);
    fmPrefs.putBytes("b", blob, 20);
  }
}

// ---------- الحساس ----------
uint32_t tReq = 0;
bool pending = false;
uint8_t lastSentState = 255;
int8_t lastSentLock = -1;

void onTemp(uint32_t now, float t) {
  char b[32];
  if (t <= -100.0f) {                 // الحساس غير متصل (-127)
    Serial.println("DS18B20 not found");
    if (connected) sendLine("Temp: ERR\n");
    return;
  }
  if (t == 85.0f) return;             // قيمة التشغيل الأولى
  lastTemp = t;
  mhe.feedTemp(now, t);

  // 3 قراءات متتالية فوق حد الخطر لتفعيل الحماية، والنزول تحت حد التحذير لإلغائها
  if (t >= hotC) { if (hotCnt < 255) hotCnt++; if (hotCnt >= 3) lockOn = true; }
  else hotCnt = 0;
  hardStop = (t >= hotC + 5.0f);
  if (lockOn && t < effWarn()) lockOn = false;

  snprintf(b, sizeof b, "Temp: %.2f C\n", t);
  Serial.print(b);
  if (connected) sendLine(b);
}

void sendStatus() {
  static const char* names[] = {"OFF", "SCAN", "ON", "FAIL"};
  char b[32];
  uint8_t st = guardState;
  snprintf(b, sizeof b, "ST:%s\nLK:%d\n", names[st > 3 ? 0 : st], lockOn ? 1 : 0);
  sendLine(b);
  lastSentState = st;
  lastSentLock = lockOn ? 1 : 0;
}

void setup() {
  Serial.begin(115200);
  ds.begin();
  ds.setResolution(12);
  ds.setWaitForConversion(false);

  mhPrefs.begin("mh", false);
  mhe.begin(mhLoad, mhSave, mhTx, mhSleep, 20);

  limPrefs.begin("lim", false);
  warnC = limPrefs.getFloat("w", 60.0f);
  hotC  = limPrefs.getFloat("h", 80.0f);
  if (warnC < 20 || warnC > 120) warnC = 60;
  if (hotC < 20 || hotC > 120) hotC = 80;

  fmPrefs.begin("fm", false);
  {
    uint8_t blob[20];
    if (fmPrefs.getBytes("b", blob, 20) == 20) {
      memcpy(fmFeatB, blob, 8); memcpy(fmSpdB, blob + 8, 6); memcpy(fmIncB, blob + 14, 6);
    }
  }

  NimBLEDevice::init(DEV_NAME);
  NimBLEDevice::setMTU(517);
  gServer = NimBLEDevice::createServer();
  gServer->setCallbacks(new SrvCb());

  NimBLEService* svc = gServer->createService(NUS_SVC);
  txChar = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  txChar->setCallbacks(new TxCb());
  NimBLECharacteristic* rx = svc->createCharacteristic(
      NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();

  NimBLEService* osvc = gServer->createService(OTA_SVC);
  otaCtl = osvc->createCharacteristic(OTA_CTL, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  otaCtl->setCallbacks(new OtaCtlCb());
  NimBLECharacteristic* otaDat = osvc->createCharacteristic(
      OTA_DAT, NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE);
  otaDat->setCallbacks(new OtaDatCb());
  osvc->start();

  NimBLEService* fsvc = gServer->createService(U16(0x1826));
  fmFeat = fsvc->createCharacteristic(U16(0x2ACC), NIMBLE_PROPERTY::READ);
  fmData = fsvc->createCharacteristic(U16(0x2ACD), NIMBLE_PROPERTY::NOTIFY);
  fmData->setCallbacks(new FmSubCb());
  fmSpd = fsvc->createCharacteristic(U16(0x2AD4), NIMBLE_PROPERTY::READ);
  fmInc = fsvc->createCharacteristic(U16(0x2AD5), NIMBLE_PROPERTY::READ);
  fmStatus = fsvc->createCharacteristic(U16(0x2ADA), NIMBLE_PROPERTY::NOTIFY);
  fmCp = fsvc->createCharacteristic(U16(0x2AD9),
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::INDICATE);
  fmCp->setCallbacks(new FmCpCb());
  fmFeat->setValue(fmFeatB, 8); fmSpd->setValue(fmSpdB, 6); fmInc->setValue(fmIncB, 6);
  fsvc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SVC);
  adv->addServiceUUID(U16(0x1826));
  adv->setScanResponse(true);
  adv->start();

  xTaskCreate(guardTask, "guard", 8192, nullptr, 1, nullptr);
}

void loop() {
  uint32_t now = millis();

  if (pendW > 0) { warnC = pendW; pendW = 0; limPrefs.putFloat("w", warnC); }
  if (pendH > 0) { hotC = pendH; pendH = 0; limPrefs.putFloat("h", hotC); }

  if (!pending && now - tReq >= 2000) { ds.requestTemperatures(); tReq = now; pending = true; }
  if (pending && now - tReq >= 800)   { pending = false; onTemp(now, ds.getTempCByIndex(0)); }

  static uint32_t lastChk = 0;
  if ((uint32_t)(now - lastChk) >= 1000) {            // لا أحد متصل: صفّر الحالات
    lastChk = now;
    if (gServer->getConnectedCount() == 0) connected = false;
  }

  static uint32_t lastClientMs = 0;
  if (gServer->getConnectedCount() > 0) lastClientMs = now;
  bool gone = (uint32_t)(now - lastClientMs) > IDLE_MS;
  guardWant = wantLink(now, lockOn, manualLink, ftmsLastAct, dOffAt, lastTemp, warnC, gone);

  if (connected && now - connMs > 1500) {
    if (needStatus || lastSentState != guardState || lastSentLock != (lockOn ? 1 : 0)) {
      needStatus = false;
      sendStatus();
    }
  }
  bridgePump(now);
  otaPump();
  mhe.loop(now);
  delay(5);
}
