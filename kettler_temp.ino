// ============================================================================
// كود لوحة Kettler-Temp المخصص لشريحة ESP32-S3 N16 (16MB Flash / Dual Core LX7)
// ----------------------------------------------------------------------------
// إعدادات البناء والترجمة في Arduino IDE:
//   - Board: "ESP32S3 Dev Module"
//   - Flash Size: "16MB (128Mb)"
//   - Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)" أو "app3M_fat9M_16MB"
//   - Flash Mode: "QIO 80MHz" أو "DIO 80MHz"
//   - PSRAM: "OPI PSRAM" (إذا كانت N16R8) أو "Disabled" (إذا كانت N16 فقط)
//   - CPU Frequency: "240MHz (WiFi/BT)"
//   - Upload Speed: 921600
// ----------------------------------------------------------------------------
// توزيع الموارد على الشريحة:
//   - Core 0: معالجة الـ DSP وتيار ACS758 بسرعة 8.33 kHz (dspTask)
//   - Core 1: بروتوكول البلوتوث (NimBLE Stack) ومراقبة الحماية وقراءة الحرارة
//   - ADC1 (GPIO 5): قراءة مستمرة ومستقلة تماماً لا تتأثر بالبلوتوث إطلاقاً
// ============================================================================
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
#include "MotorCurrentDSP.h"

#define PIN_DS   4
#define PIN_ACS  5
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

// تصريحات مسبقة لضمان الترجمة القياسية في مترجم C++ و Arduino CLI
void sendLine(const char* s);
void sendLong(const char* s);
void dspSave();
void dspSend();
void sendStatus();
volatile bool rawDumpReq = false;
void rawDumpPump();

// إعدادات تُضبط من التطبيق وتُحفظ في ذاكرة اللوحة
struct Cfg {
  float maxSpd;     // أقصى سرعة مسموحة كم/س (0 = حد المشاية نفسها)
  float maxInc;     // أقصى ميل مسموح % (0 = حد المشاية)
  float accel;      // أقصى تسارع كم/س في الثانية (0 = بلا حد): بداية آمنة
  float decel;      // أقصى تباطؤ كم/س في الثانية (0 = بلا حد)
  float capMin;     // أدنى سرعة مسموحة عند بلوغ حد الخطر
  uint8_t capOn;    // تحديد السرعة بالحرارة بين التحذير والخطر
  uint8_t cdMin;    // دقائق المشي البطيء قبل الإيقاف عند أمر الإيقاف (0 = إيقاف مباشر)
};

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
Preferences cfgPrefs;
Preferences dspPrefs;
volatile bool dspDirty = false, dspQuery = false;
volatile uint32_t dspDirtyMs = 0;
Cfg cfg = {0.0f, 0.0f, 0.5f, 1.0f, 3.0f, 1, 0};
volatile bool cfgDirty = false, cfgQuery = false;
volatile uint32_t cfgDirtyMs = 0;
mh::Engine mhe;
dsp::MotorCurrentDSP dspEngine;

// Live speed from treadmill bridge (forward-declared for dspTask)
volatile float tmSpeed = 0, tmIncl = 0;
volatile uint32_t tmSpeedMs = 0;

void dspTask(void*) {
  for (;;) {
    dspEngine.processBurst(millis(), tmSpeed, tmIncl);
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}


// ---------- تشخيص أسباب التوقف وحالة الأمان المتقدمة ----------
enum StopReasonCode : uint8_t {
  STOP_NONE = 0,             // طبيعي / لا يوجد خطأ
  STOP_USER_CONSOLE = 1,     // توقف عادي من أزرار كونسول المشاية
  STOP_SAFETY_KEY = 2,       // فصل مفتاح الأمان المغناطيسي للمشاية
  STOP_TEMP_HOT = 3,         // حماية حرارية: تجاوز 80°م
  STOP_STALL_OVERLOAD = 4,   // حماية انحشار المحرك: تيار > 15A
  STOP_TEMP_GLITCH = 5,      // اهتزاز أو انقطاع سلك حساس الحرارة
  STOP_BLE_TIMEOUT = 6,      // مهلة أمان البلوتوث / فقدان الاتصال
  STOP_MANUAL_APP = 7,       // إيقاف من تطبيق الويب
  STOP_BRUSH_WEAR = 8        // تآكل وتفحم شديد في فحمات المحرك
};

volatile uint8_t lastStopCode = STOP_NONE;
volatile uint32_t lastStopMs = 0;
char lastStopReason[48] = "NORMAL";

volatile float lastTemp = -100.0f;

// ---------- سجل آخر 5 أخطاء للأمان المحفوظة في فلاش NVS (v35.7) ----------
struct FaultRecord {
  uint8_t code;
  uint32_t timestampMs;
  float currentA;
  float tempC;
  float speedKmh;
};

FaultRecord faultHistory[5];
uint8_t faultCount = 0;
Preferences errPrefs;

void sendFaultHistory() {
  if (!connected) return;
  char b[256];
  int len = snprintf(b, sizeof(b), "FLOG:CNT=%d", faultCount);
  for (int i = 0; i < faultCount && i < 5; i++) {
    uint32_t nowMs = millis();
    uint32_t agoSec = (nowMs >= faultHistory[i].timestampMs) ? (nowMs - faultHistory[i].timestampMs) / 1000 : 0;
    len += snprintf(b + len, sizeof(b) - len, "|%d=%d,%lu,%.1f,%.1f,%.1f",
                    i, faultHistory[i].code, (unsigned long)agoSec,
                    faultHistory[i].currentA, faultHistory[i].tempC, faultHistory[i].speedKmh);
    if (len >= (int)sizeof(b) - 30) break;
  }
  b[len++] = '\n';
  b[len] = 0;
  sendLong(b);
}

void loadFaultHistory() {
  errPrefs.begin("k_errs", false);
  faultCount = errPrefs.getUChar("cnt", 0);
  if (faultCount > 5) faultCount = 5;
  if (faultCount > 0) {
    errPrefs.getBytes("recs", faultHistory, sizeof(faultHistory));
  }
  Serial.printf("[NVS] Loaded %d fault records from NVS flash.\n", faultCount);
}

void saveFault(uint8_t code) {
  if (code == STOP_NONE || code == STOP_USER_CONSOLE || code == STOP_MANUAL_APP) return;
  if (faultCount > 0 && faultHistory[0].code == code && (millis() - faultHistory[0].timestampMs) < 4000) return;

  for (int i = 4; i > 0; i--) {
    faultHistory[i] = faultHistory[i - 1];
  }
  faultHistory[0].code = code;
  faultHistory[0].timestampMs = millis();
  auto m = dspEngine.getMetrics();
  faultHistory[0].currentA = m.currentA;
  faultHistory[0].tempC = lastTemp > -50.0f ? lastTemp : 25.0f;
  faultHistory[0].speedKmh = tmSpeed;

  if (faultCount < 5) faultCount++;
  errPrefs.putUChar("cnt", faultCount);
  errPrefs.putBytes("recs", faultHistory, sizeof(faultHistory));
  Serial.printf("[NVS] Logged new safety fault: %d (Total in flash: %d)\n", code, faultCount);
  sendFaultHistory();
}

void clearFaultHistory() {
  faultCount = 0;
  memset(faultHistory, 0, sizeof(faultHistory));
  errPrefs.putUChar("cnt", 0);
  errPrefs.remove("recs");
  Serial.println("[NVS] Fault history cleared.");
  sendFaultHistory();
}

void setStopReason(uint8_t code, const char* reason) {
  lastStopCode = code;
  lastStopMs = millis();
  strncpy(lastStopReason, reason, sizeof(lastStopReason) - 1);
  lastStopReason[sizeof(lastStopReason) - 1] = 0;
  Serial.printf("[DIAG] Event Code: %d, Reason: %s\n", code, reason);
  if (connected) {
    char b[64];
    snprintf(b, sizeof b, "DIAG:C=%d,R=%s\n", code, reason);
    sendLine(b);
  }
  saveFault(code);
}

// ---------- سجل الأعطال المثبتة ومعمارية الأمان الصناعية (Latched Faults) ----------
#define FAULT_NONE          0x00
#define FAULT_STALL_LATCH   0x01   // عطل انحشار التيار (لا يُمحى إلا بعد التوقف التام للسير)
#define FAULT_TEMP_HOT      0x02   // عطل حرارة المحرك فوق 80°م
#define FAULT_TEMP_CRIT     0x04   // حرارة حرجة فوق 85°م (إيقاف فوري)
#define FAULT_TEMP_DISC     0x08   // انقطاع حساس الحرارة أثناء تشغيل المحرك

volatile uint8_t latchedFaults = FAULT_NONE;

volatile bool lockOn = false;        // حماية الحرارة فعّالة
volatile bool hardStop = false;      // الحرارة فوق الخطر بفارق كبير: إيقاف فوري

inline void updateFaultFlags() {
  lockOn = (latchedFaults != FAULT_NONE);
  hardStop = (latchedFaults & (FAULT_STALL_LATCH | FAULT_TEMP_CRIT)) != 0;
}

// ---------- حدود الحرارة ----------
float warnC = 60.0f, hotC = 80.0f;
volatile float pendW = 0, pendH = 0;
volatile bool manualLink = false;    // زر "ربط المشاية" في التطبيق
volatile uint32_t dOffAt = 0;        // لحظة ضغط "فصل": تُلغي أسباب الاتصال الضعيفة بعدها
// lastTemp declared above
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
  if (strcmp(l, "RAW") == 0 || strcmp(l, "CDUMP") == 0) {
    rawDumpReq = true;
    Serial.println("[CMD] Raw burst dump requested via phone (non-blocking flag set)");
    return;
  }
  if (l[0] == 'F' && l[1] == 'Q') { sendFaultHistory(); return; }
  if (l[0] == 'F' && l[1] == 'C') { clearFaultHistory(); return; }
  if (l[0] == 'C' && l[1] == 'L') { latchedFaults = FAULT_NONE; updateFaultFlags(); sendStatus(); Serial.println("[SAFETY] Faults cleared via command."); return; }
  if ((l[0] == 'I' || l[0] == 'K') && l[1] == 'Z') {
    dspEngine.calibrateZero(200);
    dspDirty = true;
    dspDirtyMs = millis();
    auto m = dspEngine.getMetrics();
    Serial.printf("[CALIB] Zero calibration complete. New V0 = %.1f mV\n", m.zeroVoltMv);
    if (connected) {
      char b[64];
      snprintf(b, sizeof b, "IZ:OK=%.1f\n", m.zeroVoltMv);
      sendLine(b);
    }
    return;
  }
  if (l[0] == 'I' && l[1] == 'S') { dspEngine.resetSteps(); return; }
  if (l[0] == 'I' && l[1] == 'A') { dspEngine.resetArcingScore(); return; }
  if ((l[0] == 'M' && l[1] == 'M') || (l[0] == 'L' && l[1] == 'M')) {
    dspEngine.onMaintenanceLubricated();
    dspDirty = true;
    dspDirtyMs = millis();
    char mmCmd[4] = "MM";
    mhe.pushCommand(mmCmd);
    sendLine("LM:OK\n");
    Serial.println("[LUBE] Maintenance reset & empty belt baseline calibration triggered.");
    return;
  }
  if (l[0] == 'M') {
    mhe.pushCommand(l);
    return;
  }
  if (l[0] == 'W') { float v = atof(l + 1); if (v >= 20 && v <= 120) pendW = v; return; }
  if (l[0] == 'H') { float v = atof(l + 1); if (v >= 20 && v <= 120) pendH = v; return; }
  if (l[0] == 'K') {                       // إعدادات: KS سرعة، KI ميل، KA تسارع، KD تباطؤ، KT تحديد بالحرارة، KC دقائق المشي، KQ استعلام
    float v = atof(l + 2);
    switch (l[1]) {
      case 'S': cfg.maxSpd = v < 0 ? 0 : (v > 40 ? 40 : v); cfgDirty = true; cfgDirtyMs = millis(); break;
      case 'I': cfg.maxInc = v < 0 ? 0 : (v > 40 ? 40 : v); cfgDirty = true; cfgDirtyMs = millis(); break;
      case 'A': cfg.accel = v < 0 ? 0 : (v > 5 ? 5 : v); cfgDirty = true; cfgDirtyMs = millis(); break;
      case 'D': cfg.decel = v < 0 ? 0 : (v > 10 ? 10 : v); cfgDirty = true; cfgDirtyMs = millis(); break;
      case 'T': cfg.capOn = v >= 0.5f ? 1 : 0; cfgDirty = true; cfgDirtyMs = millis(); break;
      case 'C': cfg.cdMin = (uint8_t)(v < 0 ? 0 : (v > 30 ? 30 : v)); cfgDirty = true; cfgDirtyMs = millis(); break;
      case 'Q': cfgQuery = true; break;
    }
    return;
  }
  if (l[0] == 'C' && l[1] == 'W') { // CW<weight>: معايرة ذكية حية للوزن بنقرة واحدة
    float userWeight = atof(l + 2);
    if (userWeight >= 30.0f && userWeight <= 160.0f) {
      auto m = dspEngine.getMetrics();
      float emptyRef = m.emptyBeltA > 0.5f ? m.emptyBeltA : 1.08f;
      float deltaA = m.currentA - emptyRef;
      if (deltaA >= 0.40f) {
        auto c = dspEngine.getConfig();
        c.weightKgPerA = constrain(userWeight / deltaA, 15.0f, 85.0f);
        dspEngine.setConfig(c);
        dspDirty = true;
        dspDirtyMs = millis();
        Serial.printf("[CALIB] Weight factor calibrated to %.2f kg/A for user %.1f kg (delta: %.2f A)\n", c.weightKgPerA, userWeight, deltaA);
        if (connected) {
          char b[64];
          snprintf(b, sizeof b, "CW:OK=%.1f,K=%.2f\n", userWeight, c.weightKgPerA);
          sendLine(b);
        }
      }
    }
    return;
  }
  if (l[0] == 'P') { // ضبط هوامش ومعايرة الـ DSP: PF, PL, PD, PK, PW, PC, PB, PT, PS, PV, PQ, PR
    if (l[1] == 'Q') { dspQuery = true; return; }
    if (l[1] == 'R') { dspEngine.resetConfigToDefaults(); dspDirty = true; dspDirtyMs = millis(); return; }
    float v = atof(l + 2);
    auto c = dspEngine.getConfig();
    switch (l[1]) {
      case 'F': c.ratedCurrentA = constrain(v, 5.0f, 25.0f); break;
      case 'L': c.stallLimitA   = constrain(v, 2.0f, 15.0f); break;
      case 'D': c.stallDelayMs  = (uint16_t)constrain(v, 100.0f, 1500.0f); break;
      case 'K': c.kurtosisThresh = constrain(v, 4.0f, 30.0f); break;
      case 'W': c.cusumWarnScore = constrain(v, 5.0f, 50.0f); break;
      case 'C': c.cusumCritScore = constrain(v, 20.0f, 90.0f); break;
      case 'B': c.emptyBeltWarnA = constrain(v, 1.5f, 6.0f); break;
      case 'T': c.stepDeltaA     = constrain(v, 0.08f, 0.80f); break;
      case 'S': c.sensMvPerA     = constrain(v, 15.0f, 45.0f); break;
      case 'V': c.nominalBusV    = constrain(v, 90.0f, 240.0f); break;
      case 'E': c.autoSensOn     = v >= 0.5f ? 1 : 0; break;
      case 'J': c.inertiaDelayS  = constrain(v, 1.0f, 10.0f); break;
      case 'N': c.inclineCompA   = constrain(v, 0.0f, 0.50f); break;
      case 'Z': c.zeroTrackingOn = v >= 0.5f ? 1 : 0; break;
      case 'X': c.weightKgPerA = constrain(v, 15.0f, 90.0f); break;
    }
    dspEngine.setConfig(c);
    dspDirty = true;
    dspDirtyMs = millis();
    return;
  }
  if (l[0] == 'B') { // أوامر النسخ الاحتياطي والاستعادة لحماية العدادات والمعايرات
    if (l[1] == 'K' && l[2] == 'Q') { // BKQ: طلب تصدير النسخة الاحتياطية الشاملة
      char b[160];
      auto c = dspEngine.getConfig();
      snprintf(b, sizeof b, "BKP:H=%lu,M=%lu,N=%u,E=%.2f,S=%lu,BL=%.2f,STL=%.1f,W=%.1f,K=%.1f\n",
               (unsigned long)mhe.st.totalSec, (unsigned long)mhe.st.maintSec,
               (unsigned)mhe.st.count, dspEngine.getCumulativeEnergyKWh(),
               (unsigned long)dspEngine.getCumulativeSteps(),
               c.calibratedBaselineA, c.stallLimitA, warnC, hotC);
      sendLong(b);
      return;
    }
    if (l[1] == 'K' && l[2] == 'R') { // BKR: استعادة العدادات والمعايرة
      const char* p = l + 4;
      float ekwh = -1.0f; long steps = -1; float bl = -1.0f;
      unsigned long totSec = 0, maintSec = 0;
      char copy[128]; strncpy(copy, p, sizeof copy - 1); copy[sizeof copy - 1] = 0;
      char* tok = strtok(copy, ",");
      while (tok) {
        if (tok[0] == 'H' && tok[1] == '=') totSec = strtoul(tok + 2, 0, 10);
        else if (tok[0] == 'M' && tok[1] == '=') maintSec = strtoul(tok + 2, 0, 10);
        else if (tok[0] == 'E' && tok[1] == '=') ekwh = atof(tok + 2);
        else if (tok[0] == 'S' && tok[1] == '=') steps = strtol(tok + 2, 0, 10);
        else if (tok[0] == 'B' && tok[1] == 'L' && tok[2] == '=') bl = atof(tok + 3);
        tok = strtok(0, ",");
      }
      if (totSec > 0) mhe.st.totalSec = totSec;
      if (maintSec > 0) mhe.st.maintSec = maintSec;
      mhSave(&mhe.st, sizeof(mhe.st));
      if (ekwh >= 0.0f || steps >= 0) {
        float curE = ekwh >= 0.0f ? ekwh : dspEngine.getCumulativeEnergyKWh();
        uint32_t curS = steps >= 0 ? (uint32_t)steps : dspEngine.getCumulativeSteps();
        dspEngine.loadCumulativeCounters(curE, curS);
      }
      if (bl >= 0.5f && bl <= 5.0f) {
        auto c = dspEngine.getConfig();
        c.calibratedBaselineA = bl;
        c.emptyBeltWarnA = bl * 1.55f;
        dspEngine.setConfig(c);
      }
      dspSave();
      sendLine("BKR:OK\n");
      return;
    }
  }
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
    if (connected) {
      connMs = millis(); needStatus = true; lineLen = 0; sendFaultHistory();
    } else {
      manualLink = false; // فصل المشاية تلقائياً فور إغلاق تطبيق الويب
    }
  }
};
extern volatile uint32_t ftmsLastAct;
class SrvCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) { if (s->getConnectedCount() < 2) NimBLEDevice::startAdvertising(); }
  void onDisconnect(NimBLEServer* s) {
    if (txChar) connected = (txChar->getSubscribedCount() > 0);
    if (!connected) manualLink = false;
    if (!s || s->getConnectedCount() == 0) {
      manualLink = false;
      ftmsLastAct = 0;
    }
    NimBLEDevice::startAdvertising();
  }
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
extern volatile bool tmLinked;       // إعلان مسبق للارتباط بالمشاية لتفادي خطأ البناء

// نتيجة أمر التحكم: 1 نجاح، 2 غير مدعوم، 5 غير مسموح (الحماية فعّالة)
uint8_t cpDecide(uint8_t op, bool lock, bool* fwd) {
  *fwd = false;
  if (op == 0x00 || op == 0x01) return 1; // قبول طلب التحكم المبدئي من Kinomap ريثما يكتمل ربط الوسيط بالمشاية
  if (!tmLinked) {
    return 5; // التحكّم غير مسموح لأن المشاية الحقيقية غير متصلة بالوسيط بعد
  }
  switch (op) {
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
  void onSubscribe(NimBLECharacteristic*, ble_gap_conn_desc*, uint16_t sub) {
    if (sub > 0) { ftmsLastAct = millis(); dOffAt = 0; }
  }
};
class FmSubCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, ble_gap_conn_desc*, uint16_t sub) {
    if (sub > 0) { ftmsLastAct = millis(); dOffAt = 0; }
  }
};

// ---------- الاتصال بالمشاية الحقيقية (مهمة منفصلة) ----------
// 0 غير متصلة، 1 بحث، 2 متصلة، 3 فشل
volatile uint8_t guardState = 0;
volatile bool guardWant = false;
volatile bool tmLinked = false;
volatile bool needCtrl = true;
NimBLEClient* tmClient = nullptr;
NimBLERemoteCharacteristic* tmCp = nullptr;
NimBLEAddress tmAddr;
volatile bool tmFound = false;
static const uint8_t CMD_REQ[] = {0x00};          // Request Control

static portMUX_TYPE pktMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t tmPkt[32]; volatile uint8_t tmPktLen = 0; volatile bool tmPktNew = false;
uint8_t tmSt[16];  volatile uint8_t tmStLen = 0;  volatile bool tmStNew = false;

void tmDataNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n < 2 || n > sizeof tmPkt) return;
  portENTER_CRITICAL(&pktMux);
  memcpy(tmPkt, d, n); tmPktLen = (uint8_t)n; tmPktNew = true;
  portEXIT_CRITICAL(&pktMux);
  uint16_t f = d[0] | (d[1] << 8);
  size_t o = 2;
  if (!(f & 1)) { if (n >= o + 2) { tmSpeed = (d[o] | (d[o + 1] << 8)) * 0.01f; tmSpeedMs = millis(); } o += 2; }
  if (f & 2) o += 2;
  if (f & 4) o += 3;
  if (f & 8) { if (n >= o + 2) tmIncl = (int16_t)(d[o] | (d[o + 1] << 8)) * 0.1f; }
}
void tmStatusNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n < 1 || n > sizeof tmSt) return;
  portENTER_CRITICAL(&pktMux);
  memcpy(tmSt, d, n); tmStLen = (uint8_t)n; tmStNew = true;
  portEXIT_CRITICAL(&pktMux);
}
void cpNotify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n >= 3 && d[0] == 0x80 && d[2] == 0x05) needCtrl = true;   // 5 = لا صلاحية تحكم
}

class ScanCb : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* d) {
    std::string nm = d->getName();
    for (size_t i = 0; i < nm.size(); i++) nm[i] = toupper((unsigned char)nm[i]);
    // استبعاد اللوحة نفسها قطعيًا لتفادي محاولة الاتصال بنفسها والتأخير
    if (nm.find("TEMP") != std::string::npos) return;

    bool ok = d->isAdvertisingService(U16(0x1826)) || (nm.find("KETTLER") != std::string::npos);
    if (ok) {
      tmAddr = d->getAddress();
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
  if (!guardWant) return false;
  guardState = 1;
  tmFound = false;
  NimBLEScan* sc = NimBLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(&scanCb, false);
  sc->setActiveScan(true);
  sc->setInterval(100);
  sc->setWindow(80);
  sc->start(3, false); // 3 ثوانٍ تكفي تمامًا لالتقاط المشاية بسرعة
  sc->clearResults();
  if (!tmFound || !guardWant) return false;
  if (!tmClient) { 
    tmClient = NimBLEDevice::createClient(); 
    tmClient->setConnectTimeout(8); 
    tmClient->setConnectionParams(24, 40, 0, 500); // 30-50ms interval, 5000ms supervision timeout
  }
  if (!tmClient->connect(tmAddr)) return false;
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
  if (needCtrl && tmCp) {
    tmCp->writeValue(CMD_REQ, 1, false);
    needCtrl = false;
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// ---------- حدود المشاية الحقيقية وحدود المستخدم ----------
float tmMaxSpeed() { return (fmSpdB[2] | (fmSpdB[3] << 8)) * 0.01f; }
float tmMinSpeed() { return (fmSpdB[0] | (fmSpdB[1] << 8)) * 0.01f; }
float tmMaxIncl()  { return (int16_t)(fmIncB[2] | (fmIncB[3] << 8)) * 0.1f; }
float effMaxSpeed() { float v = tmMaxSpeed(); if (v < 1.0f) v = 20.0f; if (cfg.maxSpd > 0 && cfg.maxSpd < v) v = cfg.maxSpd; return v; }
float effMaxIncl()  { float v = tmMaxIncl(); if (v < 0.5f) v = 15.0f; if (cfg.maxInc > 0 && cfg.maxInc < v) v = cfg.maxInc; return v; }

// أقصى سرعة مسموحة حسب الحرارة: كاملة تحت التحذير، وتنخفض خطيًا إلى vmin عند الخطر
float capSpeedFor(float temp, float warn, float hot, float vmax, float vmin, bool on) {
  if (!on || temp <= -50.0f || temp < warn) return vmax;
  if (hot <= warn || temp >= hot) return vmin < vmax ? vmin : vmax;
  float k = (temp - warn) / (hot - warn);
  float v = vmax - (vmax - vmin) * k;
  return v < vmin ? vmin : v;
}
// خطوة تسارع/تباطؤ محدودة (dt بالثواني)
float shapeStep(float out, float target, float accel, float decel, float dt) {
  if (target > out) { if (accel <= 0) return target; float m = accel * dt; return (target - out) > m ? out + m : target; }
  if (decel <= 0) return target;
  float m = decel * dt;
  return (out - target) > m ? out - m : target;
}
// الوقت المتوقع (بالدقائق) لبلوغ حد معيّن من ميل منحنى الحرارة؛ -1 إن لم ترتفع الحرارة بوضوح
float predictEta(const float* y, int n, float stepMin, float temp, float limit) {
  if (n < 24) return -1.0f;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < n; i++) { double x = i * (double)stepMin; sx += x; sy += y[i]; sxx += x * x; sxy += x * y[i]; }
  double den = n * sxx - sx * sx;
  if (den <= 0) return -1.0f;
  double slope = (n * sxy - sx * sy) / den;           // درجة/دقيقة
  if (slope < 0.05) return -1.0f;
  double eta = (limit - temp) / slope;
  return (float)(eta < 0 ? 0 : eta);
}

// ---------- شكّل أوامر التطبيقات قبل تمريرها للمشاية ----------
float reqSpeed = -1.0f, outSpeed = -1.0f, lastSentSpd = -1.0f;   // كم/س، -1 = لا شيء
bool cdActive = false;
uint32_t cdEnd = 0;
bool speedFresh() { return (uint32_t)(millis() - tmSpeedMs) < 4000; }
void sendRaw(const uint8_t* d, size_t n) {
  if (!tmCp) return;
  ensureCtrl();
  tmCp->writeValue(d, n, false);
  vTaskDelay(pdMS_TO_TICKS(40));
}
void shaperReset() { reqSpeed = -1.0f; outSpeed = -1.0f; lastSentSpd = -1.0f; cdActive = false; }

void handleFwd(const Fwd& f) {
  uint8_t op = f.d[0];
  if (op == 0x02 && f.n >= 3) {                       // سرعة مستهدفة
    cdActive = false;
    float v = (f.d[1] | (f.d[2] << 8)) * 0.01f;
    float mx = effMaxSpeed(), mn = tmMinSpeed();
    if (v > mx) v = mx;
    if (v < mn && v > 0.05f) v = mn;
    reqSpeed = v;
    if (outSpeed < 0) outSpeed = speedFresh() ? tmSpeed : v;
    return;
  }
  if (op == 0x03 && f.n >= 3) {                       // ميل مستهدف: يُمرَّر بعد تحديده بالحد
    float inc = (int16_t)(f.d[1] | (f.d[2] << 8)) * 0.1f;
    float mx = effMaxIncl();
    if (inc > mx) inc = mx;
    int16_t v = (int16_t)lroundf(inc * 10.0f);
    uint8_t b[3] = {0x03, (uint8_t)(v & 255), (uint8_t)((uint16_t)v >> 8)};
    sendRaw(b, 3);
    return;
  }
  if (op == 0x07) {                                   // بدء: السرعة تصعد تدريجيًا بعده
    cdActive = false; outSpeed = -1.0f; lastSentSpd = -1.0f;
    sendRaw(f.d, f.n);
    return;
  }
  if (op == 0x08) {                                   // إيقاف
    bool real = f.n >= 2 && f.d[1] == 0x01;
    if (real && cfg.cdMin > 0 && !cdActive && speedFresh() && tmSpeed >= 5.0f) {
      cdActive = true; cdEnd = millis() + (uint32_t)cfg.cdMin * 60000UL;   // مشي بطيء ثم إيقاف؛ أمر إيقاف ثانٍ يوقف فورًا
      reqSpeed = 3.0f;
      if (outSpeed < 0) outSpeed = tmSpeed;
      return;
    }
    shaperReset();
    sendRaw(f.d, f.n);
    return;
  }
  sendRaw(f.d, f.n);
}
void flushFwd() {
  int n = 0;
  while (fwdT != fwdH && n++ < 8) {
    Fwd f = fwdQ[fwdT];
    fwdT = (fwdT + 1) & 15;
    if ((uint32_t)(millis() - f.ms) < 15000) handleFwd(f);
  }
}
void shaperTick(uint32_t now) {
  static uint32_t last = 0;
  if ((uint32_t)(now - last) < 500) return;
  float dt = (last == 0) ? 0.5f : (now - last) / 1000.0f;
  last = now;
  if (cdActive && (int32_t)(now - cdEnd) >= 0) {      // انتهى المشي البطيء: أوقف
    shaperReset();
    uint8_t st[2] = {0x08, 0x01};
    sendRaw(st, 2);
    return;
  }
  if (reqSpeed < 0) return;
  if (outSpeed < 0) outSpeed = speedFresh() ? tmSpeed : reqSpeed;
  float cap = capSpeedFor(lastTemp, effWarn(), hotC, effMaxSpeed(), cfg.capMin, cfg.capOn != 0);
  float target = reqSpeed < cap ? reqSpeed : cap;
  outSpeed = shapeStep(outSpeed, target, cfg.accel, cfg.decel, dt);
  if (lastSentSpd < 0 || fabsf(outSpeed - lastSentSpd) >= 0.05f) {
    uint16_t v = (uint16_t)lroundf(outSpeed * 100.0f);
    uint8_t b[3] = {0x02, (uint8_t)(v & 255), (uint8_t)(v >> 8)};
    sendRaw(b, 3);
    lastSentSpd = outSpeed;
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
// إصلاح: ما دام الحزام يدور وقد أمره تطبيق بعد آخر "فصل" يبقى الاتصال قائمًا؛ فبرامج التطبيق تُرسل أوامر
// قليلة (عند تغيّر المرحلة فقط)، وقطع الاتصال بعد 20 ثانية من الصمت كان يوقف المشاية أثناء البرنامج.
bool wantLink(uint32_t now, bool lock, bool manual, uint32_t ftmsAct, uint32_t dOff, float temp, float warn, bool gone, bool moving) {
  (void)now; (void)lock; (void)temp; (void)warn; (void)gone; (void)moving;
  // 1. إذا كان بلوتوث الهاتف مغلقاً أو لا يوجد أي تطبيق متصل باللوحة نهائياً:
  // يُمنع منعاً باتاً البحث عن المشاية أو الاتصال بها، ويُفصل الاتصال فوراً.
  if (!gServer || gServer->getConnectedCount() == 0) {
    manualLink = false;
    ftmsLastAct = 0;
    return false;
  }

  // 2. فحص الاشتراكات الحية الحالية لمعرفة أي تطبيق مفتوح الآن
  bool webAppOpen = (txChar && txChar->getSubscribedCount() > 0);
  bool ftmsSub = ((fmData && fmData->getSubscribedCount() > 0) ||
                  (fmCp && fmCp->getSubscribedCount() > 0));

  // إذا أُغلق تطبيق الويب، يُلغى فوراً أمر الربط اليدوي
  if (!webAppOpen && manual) {
    manualLink = false;
    manual = false;
  }

  // 3. الحالة الأولى: تطبيقنا مفتوح والمستخدم ضغط صراحةً على زر "ربط المشاية / الاتصال بالمشاية"
  if (webAppOpen && manual && dOff == 0) {
    return true;
  }

  // 4. الحالة الثانية: تطبيق خارجي مثل Kinomap أو Zwift مفتوح ومشترك حالياً في قناة FTMS
  // وبمجرد إغلاق Kinomap يسقط ftmsSub إلى false فينقطع اتصال اللوحة بالمشاية تلقائياً وفوراً
  bool ftmsAllowed = (!dOff || (ftmsAct > 0 && (int32_t)(ftmsAct - dOff) > 0));
  if (ftmsSub && ftmsAllowed) {
    if (!webAppOpen || manual) {
      return true;
    }
    if (ftmsAct > 0 && (int32_t)(ftmsAct - connMs) > 400) {
      return true;
    }
  }

  return false;
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
          if (n && tmCp) { ensureCtrl(); tmCp->writeValue(cmd, n, false); }
        }
        fwdT = fwdH;                                  // أوامر التطبيقات مرفوضة أثناء الحماية
        shaperReset();
      } else {
        ls = {0, false, 0};
        flushFwd();
        shaperTick(millis());
      }
    } else if (!want && conn) {
      tmClient->disconnect();
      guardState = 0; tmLinked = false;
      shaperReset();
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
  uint8_t bPkt[32]; uint8_t nPkt = 0;
  portENTER_CRITICAL(&pktMux);
  if (tmPktNew) {
    tmPktNew = false;
    nPkt = tmPktLen;
    memcpy(bPkt, (const void*)tmPkt, nPkt);
  }
  portEXIT_CRITICAL(&pktMux);

  if (nPkt > 0) {                                     // بيانات المشاية إلى التطبيقات
    fmData->setValue(bPkt, nPkt);
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
  uint8_t bSt[16]; uint8_t nSt = 0;
  portENTER_CRITICAL(&pktMux);
  if (tmStNew) {
    tmStNew = false;
    nSt = tmStLen;
    memcpy(bSt, (const void*)tmSt, nSt);
  }
  portEXIT_CRITICAL(&pktMux);
  if (nSt > 0) {
    fmStatus->setValue(bSt, nSt);
    fmStatus->notify();

    // فك شفرة أسباب توقف المشاية الحقيقية من كونسول Kettler عبر بروتوكول FTMS
    uint8_t op = bSt[0];
    if (op == 0x02 || op == 0x01) {
      setStopReason(STOP_USER_CONSOLE, "CONSOLE_STOP");
    } else if (op == 0x03) {
      setStopReason(STOP_SAFETY_KEY, "SAFETY_KEY_PULLED");
    } else if (op == 0x08) {
      setStopReason(STOP_BLE_TIMEOUT, "FTMS_CTRL_LOST");
    }
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

static uint8_t discCnt = 0;

void onTemp(uint32_t now, float t) {
  char b[32];
  if (t <= -100.0f) {                 // الحساس غير متصل (-127) أو سقطت قراءة عابرة بسبب تشويش المحرك
    if (discCnt < 255) discCnt++;
    bool isRunning = (dspEngine.getMetrics().currentA > 0.4f) || (tmSpeed > 0.3f);
    // تصفية النبضات العابرة: اشتراط 5 قراءات فاشلة متتالية (10 إلى 15 ثانية متواصلة) لتأكيد الانقطاع الحقيقي
    if (discCnt >= 5) {
      if (isRunning) {
        latchedFaults |= FAULT_TEMP_DISC;
        updateFaultFlags();
        setStopReason(STOP_TEMP_GLITCH, "TEMP_DISC_RUNNING");
      } else if (discCnt == 5) {
        setStopReason(STOP_TEMP_GLITCH, "TEMP_DISCONNECTED");
      }
      if (connected) sendLine("Temp: ERR\n");
    } else {
      Serial.printf("[TEMP] 1-Wire transient read drop (%u/5): ignored\n", discCnt);
    }
    return;
  }
  if (t == 85.0f) return;             // قيمة التشغيل الأولى (Power-on Reset)

  // تصفير عداد السقطات فوراً بمجرد وصول قراءة صحيحة لمنع تراكم السقطات العابرة
  discCnt = 0;

  if (latchedFaults & FAULT_TEMP_DISC) {
    latchedFaults &= ~FAULT_TEMP_DISC;
    updateFaultFlags();
    Serial.println("[SAFETY] Temp sensor recovered and verified.");
  }

  // فلتر نبضات الحرارة الكاذبة مع عداد رفض مضاد للتجميد (Anti-Freeze Glitch Filter - Argon Point 7)
  static float lastValidTemp = -100.0f;
  static uint8_t glitchRejectCnt = 0;
  if (lastValidTemp > -50.0f && fabsf(t - lastValidTemp) > 4.5f) {
    glitchRejectCnt++;
    if (glitchRejectCnt < 3) {
      Serial.printf("[TEMP] Glitch rejected (%u/2): %.2f C (previous: %.2f C)\n", glitchRejectCnt, t, lastValidTemp);
      return;
    }
    Serial.printf("[TEMP] Step change confirmed after %u consistent readings: %.2f C\n", glitchRejectCnt, t);
  }
  glitchRejectCnt = 0;
  lastValidTemp = t;
  lastTemp = t;
  mhe.feedTemp(now, t);

  // 4 قراءات متتالية مؤكدة فوق حد الخطر لتفعيل الحماية
  if (t >= hotC) {
    if (hotCnt < 255) hotCnt++;
    if (hotCnt >= 4) {
      latchedFaults |= FAULT_TEMP_HOT;
      updateFaultFlags();
      setStopReason(STOP_TEMP_HOT, "HIGH_TEMP_80C");
    }
  } else {
    hotCnt = 0;
  }

  // تأكيد حد الإيقاف الفوري (3 قراءات متتالية فوق hotC + 5)
  static uint8_t hardCnt = 0;
  if (t >= hotC + 5.0f) {
    if (hardCnt < 255) hardCnt++;
    if (hardCnt >= 3) {
      latchedFaults |= FAULT_TEMP_CRIT;
      updateFaultFlags();
      setStopReason(STOP_TEMP_HOT, "CRITICAL_TEMP_85C");
    }
  } else {
    hardCnt = 0;
    if (latchedFaults & FAULT_TEMP_CRIT) {
      latchedFaults &= ~FAULT_TEMP_CRIT;
      updateFaultFlags();
    }
  }

  // نزول الحرارة تحت حد التحذير يمسح فقط عطل الحرارة، ولا يمسح إطلاقاً عطل انحشار التيار!
  if (t < effWarn()) {
    if (latchedFaults & (FAULT_TEMP_HOT | FAULT_TEMP_CRIT)) {
      latchedFaults &= ~(FAULT_TEMP_HOT | FAULT_TEMP_CRIT);
      updateFaultFlags();
    }
  }

  snprintf(b, sizeof b, "Temp: %.2f C\n", t);
  Serial.print(b);
  if (connected) sendLine(b);
}

// إرسال سطر طويل بحزم 20 بايت (آمن مع أي MTU)
void sendLong(const char* s) {
  size_t n = strlen(s);
  for (size_t p = 0; p < n; p += 20) {
    size_t m = n - p < 20 ? n - p : 20;
    mhTx(s + p, m);
    if (p + m < n) delay(12);
  }
}
void cfgSave() { cfgPrefs.putBytes("c", &cfg, sizeof cfg); }
void cfgLoad() {
  Cfg t;
  if (cfgPrefs.getBytes("c", &t, sizeof t) != sizeof t) return;
  if (isnan(t.maxSpd) || t.maxSpd < 0 || t.maxSpd > 40) return;
  if (isnan(t.maxInc) || t.maxInc < 0 || t.maxInc > 40) return;
  if (isnan(t.accel) || t.accel < 0 || t.accel > 5) return;
  if (isnan(t.decel) || t.decel < 0 || t.decel > 10) return;
  if (isnan(t.capMin) || t.capMin < 1 || t.capMin > 10) return;
  if (t.cdMin > 30) return;
  cfg = t;
}
void cfgSend() {
  char b[96];
  snprintf(b, sizeof b, "CF:S=%.1f,I=%.1f,A=%.2f,D=%.2f,T=%d,C=%d\n", cfg.maxSpd, cfg.maxInc, cfg.accel, cfg.decel, (int)cfg.capOn, (int)cfg.cdMin);
  sendLong(b);
}
void cfgPump() {
  if (cfgQuery) {
    cfgQuery = false;
    if (cfgDirty) { cfgDirty = false; cfgSave(); }
    if (connected) cfgSend();
    return;
  }
  if (cfgDirty && (uint32_t)(millis() - cfgDirtyMs) >= 250) {
    cfgDirty = false;
    cfgSave();
    if (connected) cfgSend();
  }
}
void dspSave() {
  auto c = dspEngine.getConfig();
  dspPrefs.putBytes("c", &c, sizeof c);
  dspPrefs.putFloat("ekwh", dspEngine.getCumulativeEnergyKWh());
  dspPrefs.putUInt("steps", dspEngine.getCumulativeSteps());
}
void dspLoad() {
  dsp::DspConfig t = dspEngine.getDefaultConfig();
  size_t storedLen = dspPrefs.getBytesLength("c");
  if (storedLen > 0) {
    size_t toRead = storedLen < sizeof(t) ? storedLen : sizeof(t);
    dspPrefs.getBytes("c", &t, toRead);
    if (t.kurtosisThresh < 15.0f) t.kurtosisThresh = 18.0f; // ترقية تلقائية للعتبة لتلائم محرك كتلر
    dspEngine.setConfig(t);
  }
  float savedKWh = dspPrefs.getFloat("ekwh", 0.0f);
  uint32_t savedSteps = dspPrefs.getUInt("steps", 0);
  dspEngine.loadCumulativeCounters(savedKWh, savedSteps);
}
void dspSend() {
  char b[190];
  auto c = dspEngine.getConfig();
  snprintf(b, sizeof b, "IPC:F=%.1f,L=%.1f,D=%u,K=%.1f,W=%.1f,C=%.1f,B=%.1f,T=%.2f,S=%.1f,V=%.1f,BL=%.2f,AE=%u,IJ=%.1f,NC=%.2f,ZT=%u,WK=%.1f\n",
           c.ratedCurrentA, c.stallLimitA, (unsigned)c.stallDelayMs,
           c.kurtosisThresh, c.cusumWarnScore, c.cusumCritScore,
           c.emptyBeltWarnA, c.stepDeltaA, c.sensMvPerA, c.nominalBusV, c.calibratedBaselineA,
           (unsigned)c.autoSensOn, c.inertiaDelayS, c.inclineCompA, (unsigned)c.zeroTrackingOn, c.weightKgPerA);
  sendLong(b);
}
void dspPump() {
  if (dspQuery) {
    dspQuery = false;
    if (dspDirty) { dspDirty = false; dspSave(); }
    if (connected) dspSend();
    return;
  }
  if (dspDirty && (uint32_t)(millis() - dspDirtyMs) >= 250) {
    dspDirty = false;
    dspSave();
    if (connected) dspSend();
  }
}


// تنبؤ: ميل الحرارة آخر 6 دقائق (عينة كل 10 ثوانٍ) -> متى نبلغ حد التحذير أو الخطر
float predBuf[36]; int predN = 0; int predShown = -2; float capShown = -1.0f;
void predictPump(uint32_t now) {
  static uint32_t lastS = 0, lastRep = 0;
  if (lastTemp < -50.0f) return;
  if ((uint32_t)(now - lastS) >= 10000) {
    lastS = now;
    if (predN < 36) predBuf[predN++] = lastTemp;
    else { memmove(predBuf, predBuf + 1, 35 * sizeof(float)); predBuf[35] = lastTemp; }
  }
  if ((uint32_t)(now - lastRep) < 30000) return;
  lastRep = now;
  bool belowWarn = lastTemp < effWarn();
  float eta = predictEta(predBuf, predN, 10.0f / 60.0f, lastTemp, belowWarn ? effWarn() : hotC);
  int shown = (eta >= 0 && eta <= 30.0f) ? (int)lroundf(eta) : -1;
  if (connected && (shown >= 0 || shown != predShown)) {
    char b[24];
    if (shown >= 0) snprintf(b, sizeof b, "PR:%c,%d\n", belowWarn ? 'W' : 'H', shown); else strcpy(b, "PR:0,0\n");
    sendLong(b);
  }
  predShown = shown;
}
void capPump() {
  float vmax = effMaxSpeed();
  float cap = capSpeedFor(lastTemp, effWarn(), hotC, vmax, cfg.capMin, cfg.capOn != 0);
  float show = cap < vmax - 0.05f ? roundf(cap * 2.0f) / 2.0f : 0.0f;
  if (show != capShown) {
    if (connected) { char b[24]; snprintf(b, sizeof b, "CAP:%.1f\n", show); sendLong(b); capShown = show; }
    else capShown = -1.0f;
  }
}

void sendStatus() {
  static const char* names[] = {"OFF", "SCAN", "ON", "FAIL"};
  char b[32];
  uint8_t st = guardState;
  snprintf(b, sizeof b, "ST:%s\nLK:%d\nDIAG:%d,%s\n", names[st > 3 ? 0 : st], lockOn ? 1 : 0, lastStopCode, lastStopReason);
  sendLine(b);
  lastSentState = st;
  lastSentLock = lockOn ? 1 : 0;
  capShown = -1.0f; predShown = -2;          // أعد إرسال حالة التحديد والتنبؤ بعد كل اتصال
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_DS, INPUT_PULLUP);
  delay(50);
  ds.begin();
  ds.setResolution(12);
  ds.setWaitForConversion(false);

  mhPrefs.begin("mh", false);
  mhe.begin(mhLoad, mhSave, mhTx, mhSleep, 20);
  loadFaultHistory();

  limPrefs.begin("lim", false);
  warnC = limPrefs.getFloat("w", 60.0f);
  hotC  = limPrefs.getFloat("h", 80.0f);
  if (warnC < 20 || warnC > 120) warnC = 60;
  if (hotC < 20 || hotC > 120) hotC = 80;

  cfgPrefs.begin("cfg", false);
  cfgLoad();
  dspPrefs.begin("dsp", false);
  dspLoad();

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
  // فصل الإعلان الرئيسي عن الاستجابة لتفادي تجاوز حد 31 بايت
  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(U16(0x1826));              // FTMS للتطبيقات الرياضية
  advData.setName(DEV_NAME);                            // اسم الجهاز
  adv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanData;
  scanData.setCompleteServices(NimBLEUUID(NUS_SVC));    // خدمة UART لتطبيق الويب
  adv->setScanResponseData(scanData);
  adv->start();

  dspEngine.begin(PIN_ACS, 180.0f, 26.4f);
  xTaskCreatePinnedToCore(dspTask, "dspTask", 8192, nullptr, 1, nullptr, 0);
  xTaskCreate(guardTask, "guard", 8192, nullptr, 1, nullptr);
}


// ---------- تصدير العينات الخام 512 نقطة بسلاسة وبدون تجميد البلوتوث (Non-Blocking) ----------
void rawDumpPump() {
  if (!rawDumpReq) return;
  static int dumpIdx = -1;
  static float bCopy[512];
  static uint32_t lastChunkMs = 0;

  if (dumpIdx < 0) {
    auto m = dspEngine.getMetrics();
    char hdr[128];
    snprintf(hdr, sizeof(hdr), "RAW:START,N=512,V0=%.1f,SENS=%.2f,SPD=%.1f,IA=%.2f,K=%.2f\n",
             m.zeroVoltMv, m.activeSensMvA, tmSpeed, m.currentA, m.kurtosis);
    sendLong(hdr);
    dspEngine.copyRawBurst(bCopy, 512);
    dumpIdx = 0;
    lastChunkMs = millis();
    return;
  }

  if ((uint32_t)(millis() - lastChunkMs) < 20) return; // مهلة 20ms غير حاجزة بين الحزم
  lastChunkMs = millis();

  if (dumpIdx < 512) {
    char chunk[140];
    int pos = snprintf(chunk, sizeof(chunk), "RAW:D=");
    int end = dumpIdx + 8 < 512 ? dumpIdx + 8 : 512;
    for (int i = dumpIdx; i < end; i++) {
      pos += snprintf(chunk + pos, sizeof(chunk) - pos, "%.1f%s", bCopy[i], (i == end - 1) ? "\n" : ",");
    }
    sendLong(chunk);
    dumpIdx = end;
  } else {
    sendLine("RAW:END\n");
    Serial.println("[DSP] Raw 512-sample burst streamed smoothly without dropping BLE connection");
    dumpIdx = -1;
    rawDumpReq = false;
  }
}

void loop() {
  uint32_t now = millis();

  if (pendW > 0) { warnC = pendW; pendW = 0; limPrefs.putFloat("w", warnC); }
  if (pendH > 0) { hotC = pendH; pendH = 0; limPrefs.putFloat("h", hotC); }

  if (!pending && now - tReq >= 2000) { ds.requestTemperatures(); tReq = now; pending = true; }
  if (pending && now - tReq >= 800)   { pending = false; onTemp(now, ds.getTempCByIndex(0)); }

  static uint32_t lastChk = 0;
  if ((uint32_t)(now - lastChk) >= 1000) {            // فحص اشتراك تطبيق الويب في خدمة الحرارة
    lastChk = now;
    if (txChar) connected = (txChar->getSubscribedCount() > 0);
  }

  static uint32_t lastClientMs = 0;
  if (gServer->getConnectedCount() > 0) lastClientMs = now;
  bool gone = (uint32_t)(now - lastClientMs) > IDLE_MS;
  bool isMotorRunning = (dspEngine.getMetrics().currentA > 0.35f);
  bool moving = ((uint32_t)(now - tmSpeedMs) < 5000 && tmSpeed > 0.3f) || isMotorRunning;
  guardWant = wantLink(now, lockOn, manualLink, ftmsLastAct, dOffAt, lastTemp, warnC, gone, moving);

  if (connected && now - connMs > 1500) {
    if (needStatus || lastSentState != guardState || lastSentLock != (lockOn ? 1 : 0)) {
      needStatus = false;
      sendStatus();
    }
  }
  static uint32_t lastCurMs = 0;
  if (connected && (uint32_t)(now - lastCurMs) >= 500) {
    lastCurMs = now;
    auto m = dspEngine.getMetrics();
    char b[128];
    snprintf(b, sizeof b, "CUR:I=%.2f,K=%.2f,B=%d,W=%.1f,S=%u,P=%.1f,E=%.2f,R=%.0f\n",
             m.currentA, m.kurtosis, (int)m.brushHealth, m.powerWatts,
             (unsigned)m.stepCount, m.symmetryPct, m.emptyBeltA, m.frictionRatioPct);
    sendLine(b);
    if (m.stallAlert) {
      latchedFaults |= FAULT_STALL_LATCH;
      updateFaultFlags();
      if (m.brushHealth >= 1) {
        setStopReason(STOP_BRUSH_WEAR, "BRUSH_ARCING_OVERLOAD");
      } else {
        setStopReason(STOP_STALL_OVERLOAD, "STALL_OVERLOAD");
      }
    } else if (m.brushHealth == 2 && tmSpeed >= 0.5f) {
      setStopReason(STOP_BRUSH_WEAR, "SEVERE_BRUSH_ARCING");
    }

    // إزالة قفل الانحشار بأمان فقط بعد توقف السير التام لـ 3.5 ثانية (Safety Interlock)
    if (latchedFaults & FAULT_STALL_LATCH) {
      if (tmSpeed <= 0.05f && (now - lastStopMs) > 3500) {
        latchedFaults &= ~FAULT_STALL_LATCH;
        updateFaultFlags();
        Serial.println("[SAFETY] Stall fault latch cleared after safe belt stop.");
      }
    }
  }
  static uint32_t lastCounterSaveMs = 0;
  static bool wasMoving = false;
  bool isMoving = (tmSpeed > 0.3f);
  if (isMoving && (millis() - lastStopMs) > 5000 && lastStopCode != STOP_NONE && latchedFaults == FAULT_NONE) {
    lastStopCode = STOP_NONE;
    strncpy(lastStopReason, "NORMAL", sizeof(lastStopReason) - 1);
    needStatus = true;
  }
  if ((uint32_t)(now - lastCounterSaveMs) >= 60000 || (wasMoving && !isMoving)) {
    lastCounterSaveMs = now;
    dspPrefs.putFloat("ekwh", dspEngine.getCumulativeEnergyKWh());
    dspPrefs.putUInt("steps", dspEngine.getCumulativeSteps());
  }
  wasMoving = isMoving;

  bridgePump(now);
  cfgPump();
  dspPump();
  predictPump(now);
  capPump();
  otaPump();
  mhe.loop(now);
  rawDumpPump();
  delay(5);
}
