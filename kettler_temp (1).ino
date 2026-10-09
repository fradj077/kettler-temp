// Kettler-Temp : ESP32-S3 + DS18B20 (GPIO 4) -> BLE Nordic UART
// يرسل  "Temp: 26.44 C\n"  كل ثانيتين، ويحلل صحة المحرك (MotorHealth.h)
// الحماية من الحرارة تتم في التطبيق، اللوحة غير متصلة بالمشاية.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include "MotorHealth.h"

#define PIN_DS   4
#define NUS_SVC  "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define DEV_NAME "Kettler-Temp"

OneWire ow(PIN_DS);
DallasTemperature ds(&ow);
BLECharacteristic* txChar = nullptr;
volatile bool connected = false;
volatile bool needStatus = false;
volatile uint32_t connMs = 0;

Preferences mhPrefs;
mh::Engine mhe;

bool mhLoad(void* p, size_t n)       { return mhPrefs.getBytes("s", p, n) == n; }
void mhSave(const void* p, size_t n) { mhPrefs.putBytes("s", p, n); }
void mhSleep(uint32_t ms)            { delay(ms); }
void mhTx(const char* d, size_t n) {
  if (!connected || !txChar) return;
  txChar->setValue((uint8_t*)d, n);
  txChar->notify();
}

// ---- استقبال الأوامر (يعمل في مهمة البلوتوث: ننسخ فقط) ----
char lineBuf[48];
size_t lineLen = 0;
volatile bool wantStatus = false;

void handleLine(const char* l) {
  if (l[0] == 'M') { mhe.pushCommand(l); return; }
  if (l[0] == 'C' || l[0] == 'D') { wantStatus = true; return; }   // الوسيط غير موجود
  // W و H (حدود الحرارة) تُهمل: الحماية في التطبيق
}
void feedByte(char c) {
  if (c == '\n' || c == '\r') {
    if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
  } else if (lineLen < sizeof(lineBuf) - 1) {
    lineBuf[lineLen++] = c;
  } else {
    lineLen = 0;   // سطر طويل جدًا: تجاهله
  }
}

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    auto v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) feedByte((char)v[i]);
  }
};
class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) { connected = true; connMs = millis(); needStatus = true; lineLen = 0; }
  void onDisconnect(BLEServer*) { connected = false; BLEDevice::startAdvertising(); }
};

void sendLine(const char* s) { mhTx(s, strlen(s)); }

void setup() {
  Serial.begin(115200);
  ds.begin();
  ds.setResolution(12);
  ds.setWaitForConversion(false);

  mhPrefs.begin("mh", false);
  mhe.begin(mhLoad, mhSave, mhTx, mhSleep, 20);

  BLEDevice::init(DEV_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new SrvCb());
  BLEService* svc = server->createService(NUS_SVC);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic* rx = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SVC);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

uint32_t tReq = 0;
bool pending = false;

void onTemp(uint32_t now, float t) {
  char b[32];
  if (t <= -100.0f) {                 // الحساس غير متصل (-127)
    Serial.println("DS18B20 not found");
    if (connected) sendLine("Temp: ERR\n");
    return;
  }
  if (t == 85.0f) return;             // قيمة التشغيل الأولى، تُتجاهل
  mhe.feedTemp(now, t);
  snprintf(b, sizeof b, "Temp: %.2f C\n", t);
  Serial.print(b);
  if (connected) sendLine(b);
}

void loop() {
  uint32_t now = millis();
  if (!pending && now - tReq >= 2000) { ds.requestTemperatures(); tReq = now; pending = true; }
  if (pending && now - tReq >= 800)   { pending = false; onTemp(now, ds.getTempCByIndex(0)); }

  if (connected && (needStatus || wantStatus) && now - connMs > 1500) {
    needStatus = false; wantStatus = false;
    sendLine("ST:OFF\nLK:0\n");
  }
  mhe.loop(now);
  delay(5);
}
