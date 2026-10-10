// Kettler-Temp v2 : ESP32-S3 + DS18B20 (GPIO 4) -> BLE Nordic UART للهاتف
// + حماية على اللوحة: عند ارتفاع الحرارة تتصل بالمشاية (FTMS) وترسل أمر الإيقاف
//   وتبقى متصلة حتى تنزل الحرارة تحت حد التحذير ثم تفصل.
// حدّا الحرارة (W و H) يرسلهما التطبيق عند كل اتصال وتحفظهما اللوحة.
// + تحديث البرنامج بالبلوتوث (OTA) من تطبيق الويب، برمز تأكيد OTA_PIN.
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

OneWire ow(PIN_DS);
DallasTemperature ds(&ow);
NimBLECharacteristic* txChar = nullptr;
volatile bool connected = false;
volatile bool needStatus = false;
volatile uint32_t connMs = 0;

Preferences mhPrefs;
Preferences limPrefs;
mh::Engine mhe;

// ---------- حدود الحرارة ----------
float warnC = 60.0f, hotC = 80.0f;
volatile float pendW = 0, pendH = 0;
volatile bool lockOn = false;       // حماية الحرارة فعّالة
volatile bool manualLink = false;   // زر "ربط المشاية" في التطبيق
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

// ---------- استقبال أوامر الهاتف ----------
char lineBuf[48];
size_t lineLen = 0;
void handleLine(const char* l) {
  if (l[0] == 'M') { mhe.pushCommand(l); return; }
  if (l[0] == 'W') { float v = atof(l + 1); if (v >= 20 && v <= 120) pendW = v; return; }
  if (l[0] == 'H') { float v = atof(l + 1); if (v >= 20 && v <= 120) pendH = v; return; }
  if (l[0] == 'C') { manualLink = true; return; }
  if (l[0] == 'D') { manualLink = false; return; }
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
class SrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) { connected = true; connMs = millis(); needStatus = true; lineLen = 0; }
  void onDisconnect(NimBLEServer*) { connected = false; NimBLEDevice::startAdvertising(); }
};

// ---------- الاتصال بالمشاية وإيقافها (مهمة منفصلة) ----------
// 0 غير متصلة، 1 بحث، 2 متصلة، 3 فشل
volatile uint8_t guardState = 0;
volatile bool guardWant = false;
volatile bool needCtrl = true;
NimBLEClient* tmClient = nullptr;
NimBLERemoteCharacteristic* tmCp = nullptr;
NimBLEAddress* tmAddr = nullptr;
volatile bool tmFound = false;
static const uint8_t CMD_REQ[]  = {0x00};          // Request Control
static const uint8_t CMD_STOP[] = {0x08, 0x01};    // Stop

void cpNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n >= 3 && d[0] == 0x80 && d[2] == 0x05) needCtrl = true;   // 5 = لا صلاحية تحكم
}

class ScanCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* d) {
    bool ok = d->isAdvertisingService(NimBLEUUID((uint16_t)0x1826));
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
  if (!tmClient) tmClient = NimBLEDevice::createClient();
  if (!tmClient->connect(*tmAddr)) return false;
  NimBLERemoteService* s = tmClient->getService(NimBLEUUID((uint16_t)0x1826));
  if (!s) { tmClient->disconnect(); return false; }
  tmCp = s->getCharacteristic(NimBLEUUID((uint16_t)0x2AD9));
  if (!tmCp) { tmClient->disconnect(); return false; }
  tmCp->subscribe(false, cpNotify);    // استقبال الردود (indication)
  needCtrl = true;
  return true;
}

void guardTask(void*) {
  uint32_t lastStop = 0;
  for (;;) {
    bool want = guardWant;
    bool conn = tmClient && tmClient->isConnected();
    if (want && !conn) {
      if (tmConnect()) { guardState = 2; lastStop = 0; }
      else { guardState = 3; vTaskDelay(pdMS_TO_TICKS(3000)); }
    } else if (want && conn) {
      guardState = 2;
      if (lockOn && (uint32_t)(millis() - lastStop) > 3000) {
        lastStop = millis();
        if (needCtrl) {
          tmCp->writeValue(CMD_REQ, 1, false);
          needCtrl = false;
          vTaskDelay(pdMS_TO_TICKS(300));
        }
        tmCp->writeValue(CMD_STOP, 2, false);
      }
    } else if (!want && conn) {
      tmClient->disconnect();
      guardState = 0;
    } else {
      guardState = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

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
  mhe.feedTemp(now, t);

  // حماية الحرارة: 3 قراءات متتالية فوق الحد لتفعيلها، والنزول تحت حد التحذير لإلغائها
  if (t >= hotC) { if (hotCnt < 255) hotCnt++; if (hotCnt >= 3) lockOn = true; }
  else hotCnt = 0;
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

  NimBLEDevice::init(DEV_NAME);
  NimBLEDevice::setMTU(517);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new SrvCb());
  NimBLEService* svc = server->createService(NUS_SVC);
  txChar = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* rx = svc->createCharacteristic(
      NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  NimBLEService* osvc = server->createService(OTA_SVC);
  otaCtl = osvc->createCharacteristic(OTA_CTL, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  otaCtl->setCallbacks(new OtaCtlCb());
  NimBLECharacteristic* otaDat = osvc->createCharacteristic(
      OTA_DAT, NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::WRITE);
  otaDat->setCallbacks(new OtaDatCb());
  osvc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SVC);
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

  guardWant = lockOn || manualLink;

  if (connected && now - connMs > 1500) {
    if (needStatus || lastSentState != guardState || lastSentLock != (lockOn ? 1 : 0)) {
      needStatus = false;
      sendStatus();
    }
  }
  otaPump();
  mhe.loop(now);
  delay(5);
}
