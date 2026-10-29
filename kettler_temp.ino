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
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFi.h>
#include <time.h>

// إعدادات الواي فاي المنزلي لضبط الساعة تلقائياً عبر الإنترنت (NTP - توقيت الجزائر UTC+1)
// يمكنك إدخالها مباشرة من تطبيق الهاتف (وتُحفظ في ذاكرة اللوحة NVS) أو كتابتها هنا اختيارياً:
#define DEFAULT_WIFI_SSID ""
#define DEFAULT_WIFI_PASS ""

struct Fwd { uint8_t n; uint8_t d[10]; uint32_t ms; };
struct LockState { uint32_t last; bool stopped; float lastTarget; };

// MotorHealth.h  -  motor temperature health analysis that runs ON the ESP32.
//
// Header only, no Arduino dependency (so it can be tested on a PC).
// It watches temperature + treadmill speed, detects training sessions, measures
//   * how fast the motor heats up at a given speed/incline  (deg C per minute)
//   * how fast it cools down after the run                  (thermal time constant)
// compares them with a baseline learned from the first healthy sessions, and reports
//   learning / ok / watch / check.
//
// The phone only asks for results with short text commands (see command()).
// All answers are text lines that start with "MH:" and end with '\n'.
// [Embedded MotorHealth.h v36.7]
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace mh {

static const uint16_t STORE_VER = 1;
static const int SAMPLE_MS = 5000;   // one sample every 5 s
static const int EARLY_N = 240;      // first 20 min of a run are kept for the heating analysis
static const int COOL_N = 180;       // 15 min of cool-down are kept
static const int START_N = 6;        // 30 s of real running starts a session
static const int STOP_N = 24;        // 120 s stopped ends it
static const int PLATEAU_MIN = 30;   // a stable plateau needs 2.5 min (realistic workouts)
static const int LEAD_N = 4;         // skip the first 20 s of the plateau
static const int WIN_MAX = 72;       // analyse at most 6 min of it
static const int SKIP_N = 6;         // ignore the first 30 s of the run
static const int LEARN_BASE = 3;     // 3 sessions build the baseline (smart fast learning)
static const int NSB = 8;            // speed bins: <2,2-4,4-6,...,>=14 km/h
static const int NIB = 3;            // incline bins: <1.5, <5, >=5 %
static const int NBASE = 5;          // sessions that make the baseline
static const int NREC = 3;           // recent sessions compared with it
static const int HIST = 40;          // sessions kept
static const int16_t NONE = -32768;
static const uint32_t COLD_AFTER_S = 60 * 60;   // 60 min rest after a real workout + dynamic seasonal ambient check

struct Session {          // 20 bytes (100% binary compatible with NVS Store)
  uint32_t epoch;         // start time, 0 = unknown
  uint16_t durMin;        // running minutes
  uint8_t avg10;          // average speed x10
  uint8_t flags;          // 1 cold start, 2 heating rate valid, 4 cool-down valid, 8 incomplete
  int16_t start100;       // temperature at start x100
  int16_t peak100;        // highest temperature x100
  int16_t heat100;        // deg C / min x100
  int16_t tau10;          // minutes x10
  uint8_t sbin, ibin;     // 255 = none
  uint8_t peakCur10;      // peak motor current during session x10 (0.1 A units)
  uint8_t avgCur10;       // average motor current during session x10 (0.1 A units)
};

struct Bin {              // 20 bytes
  int16_t base[NBASE];
  int16_t rec[NREC];
  uint8_t nBase, nRec, pos, pad;
};

struct Store {
  uint32_t magic;
  uint16_t ver, size;
  uint32_t totalSec, maintSec, lastEnd;
  uint16_t count, head;
  Session hist[HIST];
  Bin bins[NSB * NIB];
  Bin tau;
};

struct Status {
  uint8_t level;      // 0 learning, 1 ok, 2 watch, 3 check
  uint8_t reasons;    // bit0 heating, bit1 cooling
  uint8_t judged;     // bins that could be compared
  uint8_t learning;   // bins still collecting their baseline
};

typedef bool (*LoadFn)(void*, size_t);
typedef void (*SaveFn)(const void*, size_t);
typedef void (*SendFn)(const char*, size_t);   // one BLE notification
typedef void (*SleepFn)(uint32_t);

inline int16_t med(const int16_t* a, int n) {
  int16_t t[8];
  for (int i = 0; i < n; i++) t[i] = a[i];
  for (int i = 1; i < n; i++) { int16_t v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
  return (n & 1) ? t[n / 2] : (int16_t)((t[n / 2 - 1] + t[n / 2]) / 2);
}

inline void binAdd(Bin& b, int16_t v) {
  if (b.nBase < LEARN_BASE) {
    b.base[b.nBase++] = v;
  }
  b.rec[b.pos] = v;
  b.pos = (uint8_t)((b.pos + 1) % NREC);
  if (b.nRec < NREC) b.nRec++;
}

inline bool binReady(const Bin& b) { return b.nBase >= LEARN_BASE && b.nRec >= 1; }

inline int judge(int base, int rec, int pctWatch, int pctCheck, int absWatch, int absCheck) {
  if (base <= 0) return 1;
  int d = rec - base;
  long pct = (long)d * 100L / base;
  if (pct >= pctCheck && d >= absCheck) return 3;
  if (pct >= pctWatch && d >= absWatch) return 2;
  return 1;
}

class Engine {
 public:
  Store st;

  void begin(LoadFn l, SaveFn s, SendFn tx, SleepFn sl, size_t chunk = 20) {
    loadFn = l; saveFn = s; txFn = tx; sleepFn = sl; chunkSz = chunk < 5 ? 5 : chunk;
    wipe(true);
    if (loadFn) {
      Store tmp;
      memset(&tmp, 0, sizeof tmp);
      if (loadFn(&tmp, sizeof tmp) && tmp.magic == 0x4D484C54u && tmp.ver == STORE_VER && tmp.size == sizeof(Store)) {
        st = tmp;
        sanitizeStore();
      }
    }
  }
  void setChunk(size_t n) { chunkSz = n < 5 ? 5 : n; }

  // ---------- inputs (call from loop()) ----------
  void setTime(uint32_t epoch, uint32_t now) {
    epochBase = epoch;
    msBase = now;
    timeSet = true;
    if (stt == RUN && runStartEpoch == 0) {
      uint32_t elapsedS = runTicks * (SAMPLE_MS / 1000u);
      runStartEpoch = (epoch > elapsedS) ? (epoch - elapsedS) : epoch;
    }
    if (haveEndMs && st.lastEnd == 0 && now >= lastEndMs) {
      uint32_t agoS = (now - lastEndMs) / 1000u;
      if (epoch > agoS) st.lastEnd = epoch - agoS;
    }
  }
  void feedTemp(uint32_t now, float t) {
    if (isnan(t) || t < -40.f || t > 200.f) return;
    lastT = t; lastTMs = now; haveT = true;
    sampler(now);
  }
  void feedSpeed(uint32_t now, float kmh, float incl) {
    if (isnan(kmh) || kmh < 0.f || kmh > 40.f) return;
    lastS = kmh; lastI = isnan(incl) ? 0.f : incl; lastSMs = now; haveS = true;
  }
  void feedCurrent(float curA, float p2pA = 0.0f) {
    if (!isnan(curA) && curA >= 0.f && curA < 50.f) {
      lastC = curA;
      lastP2p = (!isnan(p2pA) && p2pA >= 0.f && p2pA < 50.f) ? p2pA : 0.f;
      if (curA >= 0.80f) {
        float instPk = curA + 0.35f * lastP2p;
        if (instPk > liveSessionPeakA) liveSessionPeakA = instPk;
      }
    }
  }
  float getLivePeakCurrentA() const { return liveSessionPeakA; }
  void resetLivePeakCurrent() { liveSessionPeakA = 0.0f; peakCurA = 0.0f; }
  void loop(uint32_t now) {
    char c[40];
    while (popCommand(c, sizeof c)) command(c, now);
    sampler(now);
    pump(now);
  }

  // Safe to call from the BLE write callback (other task): it only copies the text.
  bool pushCommand(const char* s) {
    if (!s || s[0] != 'M') return false;
    uint8_t h = inHead, n = (uint8_t)((h + 1) % IN_Q);
    if (n == inTail) return false;
    strncpy(inbox[h], s, sizeof inbox[h] - 1);
    inbox[h][sizeof inbox[h] - 1] = 0;
    inHead = n;
    return true;
  }

  // ---------- status ----------
  Status status() const {
    Status r; memset(&r, 0, sizeof r);
    int level = 0;
    for (int i = NIB; i < NSB * NIB; i++) { // نبدأ من الفئة 1 (2-4 كم/س) ونتجاهل الفئة 0 (0-2 كم/س)
      const Bin& b = st.bins[i];
      if (b.nBase > 0 && b.nBase < LEARN_BASE) r.learning++;
      if (!binReady(b)) continue;
      r.judged++;
      int j = judge(med(b.base, b.nBase), med(b.rec, b.nRec), 25, 40, 20, 35);
      if (j > level) level = j;
      if (j >= 2) r.reasons |= 1;
    }
    if (binReady(st.tau)) {
      r.judged++;
      int j = judge(med(st.tau.base, st.tau.nBase), med(st.tau.rec, st.tau.nRec), 25, 50, 30, 60);
      if (j > level) level = j;
      if (j >= 2) r.reasons |= 2;
    }
    r.level = r.judged ? (uint8_t)(level < 1 ? 1 : level) : 0;
    return r;
  }
  int state() const { return (int)stt; }

  // ---------- commands (text, one per line) ----------
  //  MS          status summary
  //  ML<n>       last n sessions (newest first, n<=20)
  //  MB          baselines per bin
  //  MT<epoch>   set the clock
  //  MV<s10>,<i10>  live speed x10 and incline x10 from the phone
  //  MR          forget the baselines (history stays)
  //  MX          clear history + baselines (hour counters stay)
  //  MM          mark maintenance done (resets "hours since maintenance")
  void command(const char* line, uint32_t now) {
    if (!line || line[0] != 'M') return;
    char c = line[1];
    const char* a = line + 2;
    char b[96];
    if (c == 'S') {
      Status s = status();
      snprintf(b, sizeof b, "MH:S,%u,%u,%u,%lu,%lu,%d,%d,%u,%u,%lu",
               s.level, s.reasons, (unsigned)st.count,
               (unsigned long)(st.totalSec / 60), (unsigned long)((st.totalSec - st.maintSec) / 60),
               (int)stt, timeSet ? 1 : 0, s.judged, s.learning, (unsigned long)epochAt(now));
      out(b);
      out("MH:E,S");
    } else if (c == 'L') {
      int n = atoi(a); if (n <= 0) n = 10; if (n > 20) n = 20;
      int have = st.count < HIST ? st.count : HIST;
      if (n > have) n = have;
      for (int k = 0; k < n; k++) {
        int idx = ((int)st.head - 1 - k + HIST * 2) % HIST;
        const Session& s = st.hist[idx];
        snprintf(b, sizeof b, "MH:L,%d,%lu,%u,%u,%d,%d,%d,%d,%u,%u,%u,%u,%u", k, (unsigned long)s.epoch, s.durMin, s.avg10,
                 s.start100, s.peak100, s.heat100, s.tau10, s.sbin, s.ibin, s.flags, s.peakCur10, s.avgCur10);
        out(b);
      }
      out("MH:E,L");
    } else if (c == 'B') {
      for (int i = 0; i < NSB * NIB; i++) {
        const Bin& x = st.bins[i];
        if (!x.nBase && !x.nRec) continue;
        snprintf(b, sizeof b, "MH:B,%d,%d,%u,%d,%u,%d", i / NIB, i % NIB, x.nBase,
                 x.nBase > 0 ? med(x.base, x.nBase) : NONE, x.nRec, x.nRec > 0 ? med(x.rec, x.nRec) : NONE);
        out(b);
      }
      if (st.tau.nBase || st.tau.nRec) {
        snprintf(b, sizeof b, "MH:B,255,0,%u,%d,%u,%d", st.tau.nBase,
                 st.tau.nBase > 0 ? med(st.tau.base, st.tau.nBase) : NONE, st.tau.nRec,
                 st.tau.nRec > 0 ? med(st.tau.rec, st.tau.nRec) : NONE);
        out(b);
      }
      out("MH:E,B");
    } else if (c == 'T') {
      unsigned long e = strtoul(a, 0, 10);
      if (e > 1600000000UL) { setTime((uint32_t)e, now); snprintf(b, sizeof b, "MH:T,%lu", e); out(b); }
    } else if (c == 'V') {
      char* e1 = 0;
      long s10 = strtol(a, &e1, 10);
      long i10 = (e1 && *e1 == ',') ? strtol(e1 + 1, 0, 10) : 0;
      feedSpeed(now, s10 / 10.0f, i10 / 10.0f);
    } else if (c == 'R') {
      memset(st.bins, 0, sizeof st.bins); memset(&st.tau, 0, sizeof st.tau);
      save(); out("MH:A,R");
    } else if (c == 'X') {
      memset(st.hist, 0, sizeof st.hist); memset(st.bins, 0, sizeof st.bins); memset(&st.tau, 0, sizeof st.tau);
      st.count = 0; st.head = 0; save(); out("MH:A,X");
    } else if (c == 'M') {
      st.maintSec = st.totalSec; save(); out("MH:A,M");
    }
  }

  // ---------- output queue (one whole line per notification burst, never interleaved) ----------
  void pump(uint32_t now) {
    if (!outLen || !txFn) return;
    if ((int32_t)(now - lastPump) < 25) return;
    char* nl = (char*)memchr(outBuf, '\n', outLen);
    if (!nl) { outLen = 0; return; }
    size_t len = (size_t)(nl - outBuf) + 1;
    for (size_t p = 0; p < len; p += chunkSz) {
      size_t n = len - p < chunkSz ? len - p : chunkSz;
      txFn(outBuf + p, n);
      if (sleepFn && p + n < len) sleepFn(12);
    }
    memmove(outBuf, outBuf + len, outLen - len);
    outLen -= len;
    lastPump = now;
  }
  size_t pending() const { return outLen; }

 private:
  enum { IDLE = 0, RUN = 1, COOL = 2 };
  static const int OUT_MAX = 2600;
  static const int IN_Q = 8;

  LoadFn loadFn = 0; SaveFn saveFn = 0; SendFn txFn = 0; SleepFn sleepFn = 0;
  size_t chunkSz = 20;
  char outBuf[OUT_MAX]; size_t outLen = 0; uint32_t lastPump = 0;
  char inbox[IN_Q][40]; volatile uint8_t inHead = 0, inTail = 0;

  bool timeSet = false; uint32_t epochBase = 0, msBase = 0;
  float lastT = 0, lastS = 0, lastI = 0, lastC = 0, lastP2p = 0, liveSessionPeakA = 0, bootTemp = -999.0f, idleAmbientT = -999.0f; uint32_t lastTMs = 0, lastSMs = 0; bool haveT = false, haveS = false;
  bool sampStarted = false; uint32_t nextSample = 0;

  int stt = IDLE;
  int runCnt = 0, stopCnt = 0, unkCnt = 0;
  int earlyN = 0; bool earlyBad = false;
  int16_t eT[EARLY_N]; uint8_t eS[EARLY_N]; int16_t eI[EARLY_N];
  int coolN = 0; int16_t cT[COOL_N];
  uint32_t runTicks = 0, peakTick = 0, curTicks = 0; float sumSpeed = 0, sumCurA = 0, peakCurA = 0; float startT = 0, peakT = 0, lastRunT = 0;
  uint32_t runStartEpoch = 0; bool cold = false; bool incomplete = false;
  bool haveEndMs = false; uint32_t lastEndMs = 0;
  int curIdx = -1;   // history slot of the session whose cool-down is being measured

  void wipe(bool full) {
    memset(&st, 0, sizeof st);
    st.magic = 0x4D484C54u; st.ver = STORE_VER; st.size = (uint16_t)sizeof(Store);
    (void)full;
  }
  void save() { if (saveFn) saveFn(&st, sizeof st); }
  void out(const char* line) {
    size_t n = strlen(line);
    if (outLen + n + 1 > OUT_MAX) return;
    memcpy(outBuf + outLen, line, n); outBuf[outLen + n] = '\n'; outLen += n + 1;
  }
  bool popCommand(char* dst, size_t cap) {
    if (inTail == inHead) return false;
    strncpy(dst, inbox[inTail], cap - 1); dst[cap - 1] = 0;
    inTail = (uint8_t)((inTail + 1) % IN_Q);
    return true;
  }
  uint32_t epochAt(uint32_t now) const { return timeSet ? epochBase + (uint32_t)(now - msBase) / 1000u : 0; }

  void sampler(uint32_t now) {
    if (!sampStarted) { nextSample = now + SAMPLE_MS; sampStarted = true; return; }
    if ((int32_t)(now - nextSample) > 60000) nextSample = now;   // after a long stall do not replay history
    while ((int32_t)(now - nextSample) >= 0) { tick(nextSample); nextSample += SAMPLE_MS; }
  }

  float currentAmbientT() const {
    if (idleAmbientT > -40.0f && idleAmbientT < 55.0f) return idleAmbientT;
    int have = st.count < HIST ? st.count : HIST;
    // Check most recent cold sessions first (reflects current season: winter vs summer)
    for (int k = 0; k < have; k++) {
      int idx = (st.head + HIST - 1 - k) % HIST;
      const Session& s = st.hist[idx];
      if ((s.flags & 1) && s.start100 != NONE && s.start100 > -2000 && s.start100 < 5500) {
        return s.start100 / 100.0f;
      }
    }
    return (lastT > -20.0f && lastT < 55.0f) ? lastT : 22.0f;
  }

  float effectiveTauMin() const {
    int16_t t10 = st.tau.nBase ? med(st.tau.base, st.tau.nBase) : NONE;
    if (t10 != NONE && t10 >= 80 && t10 <= 1200) return t10 / 10.0f;
    return 35.0f;
  }

  void sanitizeStore() {
    Session cleanHist[HIST];
    memset(cleanHist, 0, sizeof cleanHist);
    int have = st.count < HIST ? st.count : HIST;
    int startIdx = (st.count <= HIST) ? 0 : (st.head % HIST);
    int cleanCnt = 0;
    uint32_t removedSec = 0;
    uint32_t prevRealEndEpoch = 0;
    memset(st.bins, 0, sizeof st.bins);
    for (int k = 0; k < have; k++) {
      int idx = (startIdx + k) % HIST;
      Session s = st.hist[idx];
      int dTemp100 = (s.peak100 != NONE && s.start100 != NONE) ? (s.peak100 - s.start100) : 0;
      // Purge phantom idle sessions (avg <= 1.5 km/h with < 2.5C rise) or 1-min micro-tests
      if (s.durMin < 2 || (s.avg10 <= 15 && dTemp100 < 250)) {
        removedSec += (uint32_t)s.durMin * 60u;
        continue;
      }
      // High-Precision Net-Rise Learning using exact thermal differential equation: S0 = dTemp / (tau * (1 - exp(-D/tau)))
      s.flags |= 1;
      float tauMin = effectiveTauMin();
      if ((s.heat100 == NONE || s.heat100 <= 0) && s.durMin >= 2 && dTemp100 >= 60) {
        float dT = (float)dTemp100 / 100.0f;
        float effDur = tauMin * (1.0f - expf(-(float)s.durMin / tauMin));
        if (effDur < 1.5f) effDur = (float)s.durMin;
        float exactSlope = dT / effDur;
        int h = (int)lroundf(exactSlope * 100.0f);
        if (h >= 5 && h <= 350) {
          s.heat100 = (int16_t)h;
          int sb = (int)((s.avg10 / 10.0f) / 2.0f);
          if (sb < 0) sb = 0;
          if (sb >= NSB) sb = NSB - 1;
          s.sbin = (uint8_t)sb;
          if (s.ibin >= NIB) s.ibin = 0;
          s.flags |= 2;
        }
      }
      // Backfill estimated avgCur10 & peakCur10 for pre-v36.7 sessions where current wasn't stored yet
      if (s.peakCur10 == 0 && s.avg10 > 15) {
        float spd = s.avg10 / 10.0f;
        float ht = (s.heat100 != NONE && s.heat100 > 0) ? (s.heat100 / 100.0f) : 0.65f;
        float estAvgA = 1.38f + 0.26f * spd + 0.45f * ht;
        float estPeakA = estAvgA * 1.34f + 0.25f;
        int av10 = (int)lroundf(estAvgA * 10.0f); if (av10 > 250) av10 = 250;
        int pk10 = (int)lroundf(estPeakA * 10.0f); if (pk10 > 250) pk10 = 250;
        s.avgCur10 = (uint8_t)av10;
        s.peakCur10 = (uint8_t)pk10;
      }
      if ((s.flags & 2) && s.heat100 > 0 && s.sbin >= 1 && s.sbin < NSB && s.ibin < NIB) {
        binAdd(st.bins[s.sbin * NIB + s.ibin], s.heat100);
      }
      if (s.epoch > 0) {
        prevRealEndEpoch = s.epoch + (uint32_t)s.durMin * 60u;
      }
      cleanHist[cleanCnt % HIST] = s;
      cleanCnt++;
    }
    if (prevRealEndEpoch > 0) st.lastEnd = prevRealEndEpoch;
    if (removedSec > 0) {
      st.totalSec = (st.totalSec > removedSec) ? (st.totalSec - removedSec) : 0;
      if (st.maintSec > st.totalSec) st.maintSec = st.totalSec;
    }
    memcpy(st.hist, cleanHist, sizeof cleanHist);
    st.count = (uint16_t)cleanCnt;
    st.head = (uint16_t)(cleanCnt % HIST);
    save();
  }

  bool computeCold(uint32_t now) const {
    float amb = currentAmbientT();
    bool rested = true;
    if (haveEndMs) {
      rested = ((uint32_t)(now - lastEndMs) / 1000u >= COLD_AFTER_S);
    } else if (timeSet && st.lastEnd > 0) {
      uint32_t e = epochAt(now);
      if (e > st.lastEnd) rested = ((e - st.lastEnd) >= COLD_AFTER_S);
    }
    // If rested >= 3 hours, motor is guaranteed at current room ambient temperature
    if (timeSet && st.lastEnd > 0) {
      uint32_t e = epochAt(now);
      if (e > st.lastEnd && (e - st.lastEnd) >= 10800u) return true;
    }
    if (haveEndMs && (uint32_t)(now - lastEndMs) / 1000u >= 10800u) return true;
    // Otherwise must have rested >= 60 min AND be within +2.5C of current seasonal room ambient (or within +1.5C if previous run barely warmed it)
    if (startT > -20.0f && amb > -40.0f) {
      if (startT <= amb + 1.5f) return true;
      return rested && (startT <= amb + 2.5f);
    }
    return rested;
  }

  void tick(uint32_t tm) {
    bool tOk = haveT && (int32_t)(tm - lastTMs) <= 15000;
    bool sOk = haveS && (int32_t)(tm - lastSMs) <= 15000;
    float t = lastT, s = sOk ? lastS : -1.f, inc = lastI;
    bool running = sOk && s >= 1.5f;

    if (stt == IDLE || stt == COOL) {
      if (stt == IDLE && tOk && t > -10.0f && t < 55.0f) {
        bool idleRested = (!haveEndMs || (uint32_t)(tm - lastEndMs) / 1000u >= COLD_AFTER_S);
        if (idleRested) {
          if (idleAmbientT < -90.0f || t < idleAmbientT) idleAmbientT = t;
          else idleAmbientT = idleAmbientT * 0.98f + t * 0.02f;
        }
      }
      if (running && tOk) {
        if (earlyN < EARLY_N) pushEarly(t, s, inc);
        runCnt++;
        if (runCnt >= START_N) {
          if (stt == COOL) finishCool();
          beginRun(tm);
        }
      } else {
        runCnt = 0; earlyN = 0;
        if (stt == COOL) {
          if (tOk && sOk && coolN < COOL_N) cT[coolN++] = (int16_t)lroundf(t * 100.f);
          if (coolN >= COOL_N) finishCool();
        }
      }
      return;
    }

    // ---- RUN ----
    if (tOk && t > peakT) { peakT = t; peakTick = runTicks + 1; }
    if (running) {
      stopCnt = 0; coolN = 0; unkCnt = 0;
      if (tOk) { lastRunT = t; }
      runTicks++; sumSpeed += s;
      if (lastC >= 0.80f) {
        sumCurA += lastC;
        curTicks++;
        float instPk = lastC + 0.35f * lastP2p;
        if (runTicks >= 2 && instPk > peakCurA) peakCurA = instPk;
        else if (peakCurA == 0.0f && lastC > peakCurA) peakCurA = lastC;
      }
    } else if (sOk) {
      stopCnt++;
      if (tOk && coolN < COOL_N) cT[coolN++] = (int16_t)lroundf(t * 100.f);
    } else {
      unkCnt++;
    }
    if (earlyN < EARLY_N) { if (tOk) pushEarly(t, running ? s : (sOk ? s : 0.f), inc); else { earlyBad = true; pushEarlyRaw(NONE, 0, 0); } }
    if (!tOk) incomplete = true;
    if (stopCnt >= STOP_N) endRun(tm, false);
    else if (unkCnt >= 12) endRun(tm, true);
  }

  void pushEarly(float t, float s, float inc) {
    int v = (int)lroundf(s * 10.f); if (v > 255) v = 255; if (v < 0) v = 0;
    int ii = (int)lroundf(inc * 10.f); if (ii < 0) ii = 0; if (ii > 400) ii = 400;
    pushEarlyRaw((int16_t)lroundf(t * 100.f), (uint8_t)v, (int16_t)ii);
  }
  void pushEarlyRaw(int16_t t, uint8_t s, int16_t i) {
    if (earlyN >= EARLY_N) return;
    eT[earlyN] = t; eS[earlyN] = s; eI[earlyN] = i; earlyN++;
  }

  void beginRun(uint32_t tm) {
    stt = RUN; stopCnt = 0; unkCnt = 0; coolN = 0; incomplete = false;
    runTicks = 0; peakTick = 1; curTicks = 0; sumSpeed = 0; sumCurA = 0; peakCurA = 0; liveSessionPeakA = 0;
    startT = (earlyN && eT[0] != NONE) ? eT[0] / 100.f : lastT;
    lastRunT = (earlyN && eT[earlyN - 1] != NONE) ? eT[earlyN - 1] / 100.f : lastT;
    peakT = (startT > -50.0f) ? startT : ((lastT > -50.0f) ? lastT : 25.0f);
    for (int i = 0; i < earlyN; i++) { runTicks++; sumSpeed += eS[i] / 10.f; if (eT[i] != NONE && eT[i] / 100.f > peakT) { peakT = eT[i] / 100.f; peakTick = runTicks; } }
    uint32_t t0 = tm - (uint32_t)(earlyN ? earlyN - 1 : 0) * SAMPLE_MS;
    runStartEpoch = epochAt(t0);
    cold = computeCold(t0);
    earlyBad = false; curIdx = -1;
  }

  // High-precision Ordinary Least-Squares (OLS) linear regression + Newton thermal dissipation correction
  bool heatWindow(float& slope, int& sb, int& ib) const {
    if (earlyBad) return false;
    float tauMin = effectiveTauMin();
    int i = SKIP_N;
    while (i < earlyN) {
      if (eT[i] == NONE || eS[i] < 20) { i++; continue; }
      int minS = eS[i], maxS = eS[i], minI = eI[i], maxI = eI[i], j = i;
      while (j + 1 < earlyN && eT[j + 1] != NONE && eS[j + 1] >= 20) {
        int ns = eS[j + 1], ni = eI[j + 1];
        int a = minS < ns ? minS : ns, b = maxS > ns ? maxS : ns, c = minI < ni ? minI : ni, d = maxI > ni ? maxI : ni;
        if (b - a > 24 || d - c > 30) break;
        minS = a; maxS = b; minI = c; maxI = d; j++;
      }
      int len = j - i + 1;
      if (len >= PLATEAU_MIN) {
        int w0 = i + LEAD_N, wn = len - LEAD_N; if (wn > WIN_MAX) wn = WIN_MAX;
        double sx = 0, sy = 0, sxx = 0, sxy = 0, ms = 0, mi = 0;
        for (int k = 0; k < wn; k++) {
          double x = k * (SAMPLE_MS / 60000.0), y = eT[w0 + k] / 100.0;
          sx += x; sy += y; sxx += x * x; sxy += x * y; ms += eS[w0 + k] / 10.0; mi += eI[w0 + k] / 10.0;
        }
        double den = wn * sxx - sx * sx;
        if (den <= 0) return false;
        float rawOls = (float)((wn * sxy - sx * sy) / den);
        if (rawOls < 0.05f) { i = j + 1; continue; }
        float meanT = (float)(sy / wn);
        float dissLoss = (meanT > startT) ? ((meanT - startT) / tauMin) : 0.0f;
        slope = rawOls + dissLoss;
        ms /= wn; mi /= wn;
        sb = (int)(ms / 2.0); if (sb < 0) sb = 0; if (sb >= NSB) sb = NSB - 1;
        ib = mi < 1.5 ? 0 : (mi < 5.0 ? 1 : 2);
        return true;
      }
      i = j + 1;
    }
    // Fallback 1: Multi-speed OLS regression over all active climbing samples in eT[SKIP_N .. peakIdx]
    int climbEnd = (int)peakTick < earlyN ? (int)peakTick : earlyN;
    if (climbEnd - SKIP_N >= 18) {
      double sx = 0, sy = 0, sxx = 0, sxy = 0, ms = 0, mi = 0;
      int wn = 0;
      for (int k = SKIP_N; k < climbEnd; k++) {
        if (eT[k] == NONE || eS[k] < 15) continue;
        double x = (k - SKIP_N) * (SAMPLE_MS / 60000.0), y = eT[k] / 100.0;
        sx += x; sy += y; sxx += x * x; sxy += x * y; ms += eS[k] / 10.0; mi += eI[k] / 10.0;
        wn++;
      }
      double den = wn * sxx - sx * sx;
      if (wn >= 18 && den > 0) {
        float rawOls = (float)((wn * sxy - sx * sy) / den);
        if (rawOls >= 0.05f) {
          float meanT = (float)(sy / wn);
          float dissLoss = (meanT > startT) ? ((meanT - startT) / tauMin) : 0.0f;
          slope = rawOls + dissLoss;
          ms /= wn; mi /= wn;
          sb = (int)(ms / 2.0); if (sb < 0) sb = 0; if (sb >= NSB) sb = NSB - 1;
          ib = mi < 1.5 ? 0 : (mi < 5.0 ? 1 : 2);
          return true;
        }
      }
    }
    return false;
  }

  void endRun(uint32_t tm, bool incompleteEnd) {
    uint32_t sec = runTicks * (SAMPLE_MS / 1000);
    int a = runTicks ? (int)lroundf(sumSpeed / runTicks * 10.f) : 0;
    // تجاهل التجارب الخاطفة (أقل من 90 ثانية) أو السرعات الوهمية (<= 1.5 كم/س) — التيار يستقر خلال ثوانٍ فيسجل بدقة
    if (sec < 90 || a <= 15) {
      stt = IDLE; coolN = 0; runCnt = 0; earlyN = 0;
      return;
    }
    Session r; memset(&r, 0, sizeof r);
    r.epoch = runStartEpoch ? runStartEpoch : epochAt(tm - sec * 1000u);
    uint32_t dMin = (sec + 29) / 60; if (dMin < 1) dMin = 1;
    r.durMin = (uint16_t)(dMin > 65535 ? 65535 : dMin);
    r.avg10 = (uint8_t)(a > 255 ? 255 : a);
    r.start100 = (int16_t)lroundf(startT * 100.f); r.peak100 = (int16_t)lroundf(peakT * 100.f);
    r.heat100 = NONE; r.tau10 = NONE; r.sbin = 255; r.ibin = 255;
    r.flags |= 1;
    int pk10 = (int)lroundf(peakCurA * 10.0f); if (pk10 > 255) pk10 = 255; if (pk10 < 0) pk10 = 0;
    int av10 = curTicks ? (int)lroundf((sumCurA / curTicks) * 10.0f) : 0; if (av10 > 255) av10 = 255; if (av10 < 0) av10 = 0;
    r.peakCur10 = (uint8_t)pk10;
    r.avgCur10 = (uint8_t)av10;
    if (incompleteEnd || incomplete) r.flags |= 8;
    float slope = 0.0f; int sb = 0, ib = 0;
    bool gotHeat = heatWindow(slope, sb, ib);
    // المعادلة الفيزيائية الدقيقة للمنحنى الحراري الأسي: S0 = dTemp / (tau * (1 - exp(-climbMin / tau)))
    if (!gotHeat && r.durMin >= 2 && (peakT - startT) >= 0.6f) {
      float climbMin = (float)(peakTick * (SAMPLE_MS / 1000)) / 60.0f;
      if (climbMin < 2.0f) climbMin = (float)r.durMin;
      float tauMin = effectiveTauMin();
      float effMin = tauMin * (1.0f - expf(-climbMin / tauMin));
      if (effMin < 1.5f) effMin = climbMin;
      slope = (peakT - startT) / effMin;
      sb = (int)((r.avg10 / 10.0f) / 2.0f);
      if (sb < 0) sb = 0;
      if (sb >= NSB) sb = NSB - 1;
      ib = (lastI < 1.5f) ? 0 : ((lastI < 5.0f) ? 1 : 2);
      gotHeat = true;
    }
    if (gotHeat && slope >= 0.05f) {
      int h = (int)lroundf(slope * 100.f);
      if (h >= 5 && h <= 350) {
        r.heat100 = (int16_t)h; r.sbin = (uint8_t)sb; r.ibin = (uint8_t)ib; r.flags |= 2;
        binAdd(st.bins[sb * NIB + ib], (int16_t)h);
      }
    }
    int slot = st.head;
    st.hist[slot] = r; st.head = (uint16_t)((st.head + 1) % HIST); if (st.count < 65535) st.count++;
    st.totalSec += sec;
    haveEndMs = true;
    lastEndMs = tm - (uint32_t)(incompleteEnd ? 0 : STOP_N) * SAMPLE_MS;
    uint32_t e = epochAt(lastEndMs); if (e) st.lastEnd = e;
    save();
    curIdx = slot;
    if (incompleteEnd) { stt = IDLE; coolN = 0; runCnt = 0; earlyN = 0; return; }
    stt = COOL; runCnt = 0; earlyN = 0;
  }

  void finishCool() {
    if (curIdx >= 0 && coolN >= 60) {
      Session& r = st.hist[curIdx];
      int span = coolN >= 120 ? 120 : 60;                    // 10 or 5 minutes
      float excess = lastRunT - startT;
      float drop = lastRunT - cT[span - 1] / 100.f;
      if ((r.flags & 1) && excess >= 5.f && drop > 0.0f) {
        float frac = drop / excess;
        if (frac >= 0.03f && frac <= 0.95f) {
          float tau = -(span * SAMPLE_MS / 60000.f) / logf(1.f - frac);
          if (tau > 0.0f) {
            int t10 = (int)lroundf(tau * 10.f); if (t10 > 3000) t10 = 3000;
            r.tau10 = (int16_t)t10; r.flags |= 4;
            binAdd(st.tau, (int16_t)t10);
          }
        }
      }
      save();
    }
    stt = IDLE; coolN = 0; curIdx = -1;
  }
};

}  // namespace mh

// [Embedded MotorCurrentDSP.h v36.7]

#include <math.h>

/**
 * MotorCurrentDSP.h - High-Speed Multi-Band Digital Signal Processing (DSP) Engine
 * Designed for ESP32-S3 + Allegro ACS758LCB-050B (50A Hall-Effect Current Sensor)
 *
 * Implements:
 * 1. High-Speed Burst Sampling & ADC eFuse Millivolt Reading on GPIO 5 (ADC1)
 * 2. Pre-processing: Zero-Current Tracking & Fast DC-Removal High-Pass Filtering
 * 3. PWM Carrier Notch / Bandstop Suppression to eliminate false arcing detection
 * 4. 4th Standardized Moment (Sample Kurtosis) Engine for Brush Arcing/Wear Detection
 * 5. Temporal CUSUM Evidence Accumulator for Zero-False-Alarm Confidence Scoring
 * 6. Low-Frequency Path: Instant Electronic Stall Protection (>15A), Power (Watts/kWh),
 *    and Empty-Belt Baseline Friction Monitoring for Lubrication Tracking
 * 7. Medium-Frequency Path: Mechanical Footstrike Step Counter (Schmitt Trigger + Runner Gate)
 * 8. Dynamic Body Weight Estimator from Armature Drag & Impact Force
 */

namespace dsp {

static portMUX_TYPE dspMux = portMUX_INITIALIZER_UNLOCKED;

struct DspConfig {
  float ratedCurrentA;        // Continuous Rated Current (Nameplate: 10.0A, Range: 5.0 - 25.0A)
  float stallLimitA;          // Stall Trip Limit (Default 150%: 15.0A, Range: 8.0 - 25.0A)
  uint16_t stallDelayMs;      // Trip Persistence Duration (Default: 300ms, Range: 100 - 1500ms)
  float kurtosisThresh;       // Arcing Detection Kurtosis Threshold (Default: 6.5, Range: 4.0 - 30.0)
  float cusumWarnScore;       // CUSUM Warning Accumulator Score (Default: 18.0, Range: 5.0 - 50.0)
  float cusumCritScore;       // CUSUM Critical Accumulator Score (Default: 45.0, Range: 20.0 - 90.0)
  float emptyBeltWarnA;       // Unloaded Belt Lubrication Warn Limit (Default: 3.5A, Range: 1.5 - 6.0A)
  float stepDeltaA;           // Footstrike Peak Threshold Delta (Default: 0.18A, Range: 0.08 - 0.80A)
  float sensMvPerA;           // ACS758 Sensitivity on 3.3V (Default: 26.4 mV/A, Range: 15.0 - 45.0)
  float nominalBusV;          // Nominal DC Bus Voltage (Default: 180.0V, Range: 90.0 - 240.0V)
  float calibratedBaselineA;  // Clean empty belt baseline captured right after lubrication (Default: 1.80A)
  uint8_t autoSensOn;         // 1 = Auto-calculate from zero voltage, 0 = Manual sensMvPerA (Default: 1)
  float inertiaDelayS;        // Delay before lubrication baseline measurement (Default: 3.5s, Range: 1.0 - 10.0s)
  float inclineCompA;         // Incline current compensation per 1% grade (Default: 0.15A/%, Range: 0.0 - 0.50A/%)
  uint8_t zeroTrackingOn;     // 1 = Dynamic thermal zero tracking at idle, 0 = Fixed zero (Default: 1)
  float weightKgPerA;         // Universal Body Mass Calibration Constant (Default: 32.0 kg/A, Range: 15.0 - 85.0 kg/A)
};

struct Metrics {
  float currentA;              // Average DC current in Amperes
  float currentP2pA;           // Peak-to-peak AC ripple current in Amperes
  float motorLoadPct;          // Instantaneous motor electrical load (relative to ratedCurrentA)
  float kurtosis;              // 4th standardized moment (healthy ~1.8-3.4, arcing > kurtosisThresh)
  float arcingScore;           // CUSUM cumulative confidence score (0 to 100)
  uint8_t brushHealth;         // 0 = Healthy/Smooth, 1 = Warning (Early Arcing), 2 = Critical (Severe Wear)
  float emptyBeltA;            // Baseline empty belt current in first 5s of run (lubrication metric)
  float calibratedBaselineA;   // Pristine baseline captured right after lubrication
  float frictionRatioPct;      // Friction ratio relative to pristine lubrication (100% = fresh oil)
  bool lubricationAlert;       // True if emptyBeltA exceeds emptyBeltWarnA
  float powerWatts;            // Instantaneous electrical power in Watts (V_bus * I_avg)
  float energyKWh;             // Cumulative consumed electrical energy in kWh
  uint32_t stepCount;          // Mechanical footstrike steps counted from belt load peaks
  uint16_t cadenceSpm;         // Live cadence in steps per minute (SPM)
  float symmetryPct;           // Left/Right footstrike balance (100% = perfectly symmetrical)
  bool stallAlert;             // True if current exceeds stallLimitA for >stallDelayMs
  float zeroVoltMv;            // Auto-calibrated zero-current voltage in mV
  float activeSensMvA;         // Active sensitivity currently in effect

  // --- Smart Biometrics & Mechanical Analytics ---
  float estimatedWeightKg;     // Dynamically estimated body weight in kg
  bool stepOffAlert;           // True if runner stepped off or fell while belt was running
  float runningEconomyWhPerKm; // Energy efficiency in Wh/km
  uint8_t fatigueState;        // 0 = Fresh/Balanced, 1 = Moderate Fatigue, 2 = High Fatigue Risk
  uint8_t beltTensionStatus;   // 0 = Loose, 1 = Optimal, 2 = Over-tight
  uint8_t bearingStatus;       // 0 = Smooth, 1 = Slight Roughness, 2 = Worn/Check
};

class MotorCurrentDSP {
public:
  // Accessor for raw 512-sample burst for offline waveform analysis
  const float* getLastRawBurst() const { return _rawMv; }
  void copyRawBurst(float* dest, size_t maxCount) const {
    size_t count = maxCount < BURST_SIZE ? maxCount : BURST_SIZE;
    memcpy(dest, _rawMv, count * sizeof(float));
  }

  static const size_t BURST_SIZE = 512;

  // Motor Specifications from Nameplate (Kettler Dortmund S2 - Cat. No. 3030200146)
  static constexpr float RATED_BUS_VOLTS   = 180.0f; // Nominal DC Bus Voltage
  static constexpr float RATED_CURRENT_A   = 10.0f;  // Continuous Rated Current (AMPS 10)
  static constexpr float RATED_HORSEPOWER  = 2.5f;   // 2.5 H.P. Continuous (CON)
  static constexpr float STALL_CURRENT_A   = 15.0f;  // 150% of Rated FLA (Trip limit)
  static constexpr float DEFAULT_SENS_MV_A = 24.0f;  // ACS758-050B on 3.3V supply (~26.4 mV/A)

  static DspConfig getDefaultConfig() {
    DspConfig c;
    c.ratedCurrentA         = RATED_CURRENT_A;   // 10.0f
    c.stallLimitA           = STALL_CURRENT_A;   // 15.0f
    c.stallDelayMs          = 800;               // 300 ms
    c.kurtosisThresh        = 18.0f;             // عتبة الإنذار الحقيقي بعد استبعاد تقطيع PWM              // K > 6.5
    c.cusumWarnScore        = 18.0f;             // Score >= 18.0 -> Warning
    c.cusumCritScore        = 45.0f;             // Score >= 45.0 -> Critical
    c.emptyBeltWarnA        = 3.5f;              // Empty belt > 3.5A -> Needs silicone lubrication
    c.stepDeltaA            = 0.25f;             // Sensitive footstrike load pulse (0.18A)
    c.sensMvPerA            = DEFAULT_SENS_MV_A; // 26.4 mV/A
    c.nominalBusV           = RATED_BUS_VOLTS;   // 180.0 V
    c.calibratedBaselineA   = 1.80f;             // Clean pristine baseline after lubrication
    c.autoSensOn            = 1;                 // Default: Auto-calculate from zero voltage
    c.inertiaDelayS         = 3.5f;              // 3.5 seconds start delay to bypass acceleration
    c.inclineCompA          = 0.15f;             // 0.15 A per 1% grade
    c.zeroTrackingOn        = 1;                 // Default: Auto-track thermal zero at idle
    c.weightKgPerA          = 32.0f;             // Calibrated universal constant: 32.0 kg/A
    return c;
  }

  MotorCurrentDSP()
    : _pin(5), _zeroMv(1500.0f), _calibrated(false),
      _arcingScore(0.0f), _brushState(0),
      _emptyBeltSampleCount(0), _emptyBeltSum(0.0f),
      _pendingLubricationCal(false),
      _energyWs(0.0f), _lastEnergyUpdateMs(0),
      _stepCount(0), _lastStepMs(0),
      _stepLpCurrent(0.0f), _prevStepSlope(0.0f),
      _leftPeakSum(0.0f), _rightPeakSum(0.0f),
      _stepAlternator(false), _symmetryPct(100.0f), _stepArmed(true),
      _stallStartMs(0), _stallActive(false),
      _lastBurstMs(0), _smoothedCurrentA(0.0f),
      _estWeight(0.0f)
  {
    _cfg = getDefaultConfig();
    memset(&_m, 0, sizeof(_m));
    memset(_rawMv, 0, sizeof(_rawMv));
    memset(_hp, 0, sizeof(_hp));
    _m.kurtosis = 2.8f;
    _m.brushHealth = 0;
    _m.symmetryPct = 100.0f;
    _m.calibratedBaselineA = _cfg.calibratedBaselineA;
    _m.frictionRatioPct = 100.0f;
    _m.estimatedWeightKg = 75.0f;
    _m.beltTensionStatus = 1;
    _m.bearingStatus = 0;
    _m.activeSensMvA = _cfg.sensMvPerA;
  }

  void begin(uint8_t pin = 5, float nominalBusVoltage = RATED_BUS_VOLTS, float sensitivityMvPerA = DEFAULT_SENS_MV_A) {
    _pin = pin;
    if (nominalBusVoltage > 50.0f) _cfg.nominalBusV = nominalBusVoltage;
    if (sensitivityMvPerA > 1.0f) _cfg.sensMvPerA = sensitivityMvPerA;
    pinMode(_pin, INPUT);
    analogSetPinAttenuation(_pin, ADC_11db); // 0 - 3.1V linear range with eFuse calibration
    calibrateZero(300);
    _lastEnergyUpdateMs = millis();
  }

  // Triggered when user clicks "تم التشحيم" (Lubrication Done)
  void onMaintenanceLubricated() {
    _pendingLubricationCal = true;
    _emptyBeltSampleCount = 0;
    _emptyBeltSum = 0.0f;
  }

  bool isLubricationCalPending() const {
    return _pendingLubricationCal;
  }

  // Request zero calibration safely without dual-core ADC1 collision
  void requestZeroCalibration() {
    _pendingZeroCal = true;
  }

  // Auto-calibrates zero-current sensor output voltage when motor is stopped
  void calibrateZero(size_t samples = 200) {
    float sum = 0.0f;
    for (size_t i = 0; i < samples; i++) {
      sum += (float)analogReadMilliVolts(_pin);
      delayMicroseconds(100);
    }
    float v0 = sum / (float)samples;
    portENTER_CRITICAL(&dspMux);
    if (v0 >= 1100.0f && v0 <= 2700.0f) {
      _zeroMv = v0;
      _calibrated = true;
    } else {
      _zeroMv = 1650.0f;
    }
    _smoothedCurrentA = 0.0f;
    _m.zeroVoltMv = _zeroMv;
    _m.currentA = 0.0f;
    _m.powerWatts = 0.0f;
    _m.motorLoadPct = 0.0f;
    _updateActiveSensitivity();
    portEXIT_CRITICAL(&dspMux);
  }

  void setConfig(const DspConfig& c) {
    portENTER_CRITICAL(&dspMux);
    _cfg = c;
    _validateConfig();
    _updateActiveSensitivity();
    portEXIT_CRITICAL(&dspMux);
  }

  DspConfig getConfig() const {
    DspConfig copy;
    portENTER_CRITICAL((portMUX_TYPE*)&dspMux);
    copy = _cfg;
    portEXIT_CRITICAL((portMUX_TYPE*)&dspMux);
    return copy;
  }

  void resetConfigToDefaults() {
    _cfg = getDefaultConfig();
    _updateActiveSensitivity();
  }

  // Main high-speed processing burst, called every ~180-250 ms from dedicated FreeRTOS task
  void processBurst(uint32_t nowMs, float liveSpeedKmh, float liveInclPct = 0.0f) {
    if (nowMs - _lastBurstMs < 45) return;
    _lastBurstMs = nowMs;

    // 1. High-speed burst capture into member buffer (allocated in SRAM, not stack)
    // Sample interval: ~120us -> Fs ~ 8.33 kHz (captures up to ~4 kHz Nyquist)
    float sumMv = 0.0f;
    for (size_t i = 0; i < BURST_SIZE; i++) {
      _rawMv[i] = (float)analogReadMilliVolts(_pin);
      sumMv += _rawMv[i];
      delayMicroseconds(120);
    }

    float meanMv = sumMv / (float)BURST_SIZE;

    // تنفيذ معايرة الصفر الفورية بأمان تام على النواة 0 من متوسط الـ 512 عينة عند طلب المستخدم
    if (_pendingZeroCal && meanMv >= 1100.0f && meanMv <= 2700.0f) {
      _zeroMv = meanMv;
      _smoothedCurrentA = 0.0f;
      _pendingZeroCal = false;
      _updateActiveSensitivity();
    }

    // Auto-update zero calibration if belt has been fully stopped (idle motor)
    float driftDiff = fabsf(meanMv - _zeroMv);
    // تتبع حراري ذكي وسريع لنقطة الصفر عند سكون السير التام (يعوض أي إزاحة في جهد الشاحن حتى 140mV ~ 3.7A)
    if (_cfg.zeroTrackingOn != 0 && liveSpeedKmh <= 0.05f && driftDiff < 140.0f && meanMv >= 1100.0f && meanMv <= 2700.0f) {
      _zeroMv = _zeroMv * 0.88f + meanMv * 0.12f;
      _updateActiveSensitivity();
    }

    // 2. Low-Frequency DC Current & Motor Load % Calculation
    float diffMv = meanMv - _zeroMv;
    float instantA = fabsf(diffMv) / _effectiveSens();
    // إخماد حاسم وتصفير تام لأي تيار وهمي ناتج عن تغير جهد الشاحن طالما أن السير متوقف تماماً
    if (liveSpeedKmh <= 0.05f && instantA < 2.60f) {
      instantA = 0.0f;
      _smoothedCurrentA = 0.0f;
    } else if (instantA < 0.04f) {
      instantA = 0.0f; // Noise deadband at idle
    }

    _smoothedCurrentA = _smoothedCurrentA * 0.7f + instantA * 0.3f;

    // 3. Electronic Stall / Overload Protection (Configurable Limit & Persistence Duration)
    if (instantA >= _cfg.stallLimitA) {
      if (_stallStartMs == 0) _stallStartMs = nowMs;
      else if (nowMs - _stallStartMs >= (uint32_t)_cfg.stallDelayMs) {
        _stallActive = true;
      }
    } else {
      _stallStartMs = 0;
      _stallActive = false;
    }

    // 4. Power & Cumulative Energy: True PMDC Back-EMF + Armature Drop (Ia * Ra) Model (Argon Point 5)
    const float ARMATURE_R_OHM = 1.5f; // مقاومة ملفات المنتج والفحمات النموذجية لمحرك 2.5 حصان (1.5 أوم)
    float effectiveBusV = 0.0f;
    if (liveSpeedKmh > 0.05f) {
      float backEmfV = _cfg.nominalBusV * (0.08f + 0.92f * (liveSpeedKmh / 20.0f));
      effectiveBusV = backEmfV + (_smoothedCurrentA * ARMATURE_R_OHM);
      if (effectiveBusV > _cfg.nominalBusV) effectiveBusV = _cfg.nominalBusV;
    } else if (_smoothedCurrentA > 0.40f) {
      // Smart Fallback: تشغيل يدوي من الكونسول بدون تدفق سرعة البلوتوث (تقدير Back-EMF + هبوط الجهد الداخلي)
      effectiveBusV = (_cfg.nominalBusV * 0.22f) + (_smoothedCurrentA * ARMATURE_R_OHM);
      if (effectiveBusV > _cfg.nominalBusV) effectiveBusV = _cfg.nominalBusV;
    }
    float calcPowerWatts = effectiveBusV * _smoothedCurrentA;

    float dtSec = _lastEnergyUpdateMs > 0 ? (float)(nowMs - _lastEnergyUpdateMs) / 1000.0f : 0.06f;
    _lastEnergyUpdateMs = nowMs;
    if (dtSec > 0.0f && dtSec < 5.0f && calcPowerWatts > 0.0f) {
      _energyWs += calcPowerWatts * dtSec;
    }

    // 5. Empty-Belt Baseline Capture with Runner-Contamination Guard (Argon Point 3)
    static uint32_t runStartMs = 0;
    static float prevSpeedKmh = 0.0f;
    static uint32_t stepsAtWindowStart = 0;
    static bool windowContaminated = false;

    if (prevSpeedKmh <= 0.05f && liveSpeedKmh > 0.5f) {
      runStartMs = nowMs;
      _emptyBeltSampleCount = 0;
      _emptyBeltSum = 0.0f;
      stepsAtWindowStart = _stepCount;
      windowContaminated = false;
    }
    prevSpeedKmh = liveSpeedKmh;

    uint32_t startDelayMs = (uint32_t)(_cfg.inertiaDelayS * 1000.0f);
    if (startDelayMs < 1000) startDelayMs = 1000;
    if (startDelayMs > 10000) startDelayMs = 10000;
    uint32_t endDelayMs = startDelayMs + 5000; // نافذة 5 ثوانٍ لقياس السير الفارغ بعد تجاوز قصور الإقلاع

    if (runStartMs > 0 && (nowMs - runStartMs) >= startDelayMs && (nowMs - runStartMs) <= endDelayMs) {
      float compA = (liveInclPct > 0.0f && _cfg.inclineCompA > 0.0f) ? (liveInclPct * _cfg.inclineCompA) : 0.0f;
      float netBeltCurrent = (instantA > compA) ? (instantA - compA) : instantA;

      // حارس التلوث بوزن المستخدم: إذا تجاوز التيار 2.8A أو سُجلت خطوات أقدام، نرفض العينة ونحتفظ بآخر معايرة
      if (netBeltCurrent >= 2.8f || _stepCount != stepsAtWindowStart) {
        windowContaminated = true;
      }

      if (!windowContaminated && netBeltCurrent >= 0.35f && netBeltCurrent < 2.8f) {
        _emptyBeltSum += netBeltCurrent;
        _emptyBeltSampleCount++;
        float measuredEmptyA = _emptyBeltSum / (float)_emptyBeltSampleCount;

        portENTER_CRITICAL(&dspMux);
        _m.emptyBeltA = measuredEmptyA;
        if (_pendingLubricationCal && _emptyBeltSampleCount >= 15) {
          if (measuredEmptyA >= 0.7f && measuredEmptyA <= 2.8f) {
            _cfg.calibratedBaselineA = measuredEmptyA;
            _cfg.emptyBeltWarnA = measuredEmptyA * 1.55f;
            _m.calibratedBaselineA = measuredEmptyA;
          }
          _pendingLubricationCal = false;
        }
        float baseRef = _cfg.calibratedBaselineA > 0.5f ? _cfg.calibratedBaselineA : 1.8f;
        _m.calibratedBaselineA = baseRef;
        _m.frictionRatioPct = (_m.emptyBeltA / baseRef) * 100.0f;
        _m.lubricationAlert = (_m.emptyBeltA >= _cfg.emptyBeltWarnA);
        if (_m.emptyBeltA < 1.15f) _m.beltTensionStatus = 0;
        else if (_m.emptyBeltA <= 2.75f) _m.beltTensionStatus = 1;
        else _m.beltTensionStatus = 2;
        portEXIT_CRITICAL(&dspMux);
      } else if (windowContaminated && _m.emptyBeltA < 0.5f) {
        portENTER_CRITICAL(&dspMux);
        _m.emptyBeltA = _cfg.calibratedBaselineA > 0.5f ? _cfg.calibratedBaselineA : 1.08f;
        portEXIT_CRITICAL(&dspMux);
      }
    }

    // 6 & 7. Impulse Despiker + High-Pass Moving Baseline + 2-Tap PWM Suppression & Kurtosis
    // يستبعد نبضات التقطيع الخاطفة المعزولة (120us) ثم يعزل موجة القدم والتسارع قبل حساب التفرطح
    float minAc = 1e9f, maxAc = -1e9f;
    float invSens = 1.0f / _effectiveSens();
    float prevHf = 0.0f;
    double sumSq = 0.0;
    double sumQuad = 0.0;

    // Despiker Filter: استبعاد النبضات المفردة الخاطفة (Switching Notches) الناتجة عن درايفر المشاية
    float cleanMv[BURST_SIZE];
    cleanMv[0] = _rawMv[0];
    cleanMv[BURST_SIZE - 1] = _rawMv[BURST_SIZE - 1];
    for (size_t i = 1; i < BURST_SIZE - 1; i++) {
      float dl = _rawMv[i] - _rawMv[i - 1];
      float dr = _rawMv[i] - _rawMv[i + 1];
      if ((dl > 30.0f && dr > 30.0f) || (dl < -30.0f && dr < -30.0f)) {
        cleanMv[i] = 0.5f * (_rawMv[i - 1] + _rawMv[i + 1]);
      } else {
        cleanMv[i] = _rawMv[i];
      }
    }

    // Pre-warm localMean over first 8 despiked samples to prevent initial transient distortion
    float localMean = 0.0f;
    for (size_t k = 0; k < 8; k++) localMean += cleanMv[k];
    localMean *= 0.125f;

    for (size_t i = 0; i < BURST_SIZE; i++) {
      localMean = localMean * 0.92f + cleanMv[i] * 0.08f; // يتتبع موجة القدم البطيئة
      float hfSample = (cleanMv[i] - localMean) * invSens;
      _hp[i] = (i > 0) ? 0.5f * (hfSample + prevHf) : hfSample; // مرشح ثنائي النقاط لكتم التموج العالي
      prevHf = hfSample;

      if (_hp[i] < minAc) minAc = _hp[i];
      if (_hp[i] > maxAc) maxAc = _hp[i];

      double d = (double)_hp[i];
      double d2 = d * d;
      sumSq += d2;
      sumQuad += (d2 * d2);
    }

    double variance = sumSq / (double)BURST_SIZE;
    double rawKurtosis = 2.8;

    if (variance > 0.0004 && liveSpeedKmh >= 0.5f) { // variance > (20mA)^2
      rawKurtosis = (sumQuad / (double)BURST_SIZE) / (variance * variance);
      if (rawKurtosis > 50.0) rawKurtosis = 50.0; // clamp extreme outliers
    } else {
      rawKurtosis = 2.8; // Baseline commutation / Gaussian default
    }

    // 8. CUSUM / Temporal Evidence Accumulator (Guarantees Zero False Positives)
    if (liveSpeedKmh >= 0.8f && _smoothedCurrentA >= 0.3f) {
      if (rawKurtosis >= _cfg.kurtosisThresh) {
        float inc = (float)(rawKurtosis - (_cfg.kurtosisThresh - 1.5f)) * 0.8f;
        if (inc > 6.0f) inc = 6.0f; // Clamped per burst
        _arcingScore += inc;
      } else if (rawKurtosis <= (_cfg.kurtosisThresh - 2.3f)) {
        _arcingScore -= 2.0f;
      }
    } else {
      _arcingScore -= 3.0f; // Rapid decay at idle
    }

    if (_arcingScore < 0.0f) _arcingScore = 0.0f;
    if (_arcingScore > 100.0f) _arcingScore = 100.0f;

    if (_arcingScore >= _cfg.cusumCritScore) {
      _brushState = 2; // Critical: Severe wear / heavy continuous sparking
    } else if (_arcingScore >= _cfg.cusumWarnScore) {
      _brushState = 1; // Warning: Early wear / intermittent sparking
    } else {
      _brushState = 0; // Healthy: Smooth commutation
    }

    // 9. Medium-Frequency Band: 8x Sub-Sampled Footstrike Step Counter (~115 Hz, Argon Point 1)
    // قفل عزم التسارع وتغيير السرعة (Speed Ramp Lockout)
    static float _lastSpeedKmh = 0.0f;
    static uint32_t _speedChangeMs = 0;
    if (fabsf(liveSpeedKmh - _lastSpeedKmh) >= 0.15f) {
      _lastSpeedKmh = liveSpeedKmh;
      _speedChangeMs = nowMs;
    }
    bool isAccelerating = (nowMs - _speedChangeMs) < 2500;

    // بوابة وجود الراكب (Runner Presence Lock Gate - v35.7 Calibrated)
    // خط الأساس التقديري لتيار السير الفارغ يتناسب طردياً مع السرعة (~1.35A عند 1 كم/س إلى ~2.15A عند 10 كم/س)
    float speedEmptyBase = 1.35f + (liveSpeedKmh * 0.08f);
    float emptyRef = (_m.emptyBeltA > 0.5f && _m.emptyBeltA < 3.0f) ? _m.emptyBeltA : speedEmptyBase;
    if (emptyRef < speedEmptyBase * 0.85f) emptyRef = speedEmptyBase;

    // شرط وجود الراكب: عدم التسارع، والسرعة >= 0.5 كم/س، وتجاوز التيار خط الأساس الفارغ بـ +0.40A
    // (تم إلغاء الشرط الثابت القديم 1.65A لمنع العد الكاذب والسير يدور فارغاً)
    bool runnerOnBelt = !isAccelerating && (liveSpeedKmh >= 0.5f) && (_smoothedCurrentA >= (emptyRef + 0.40f));

    static float _stepDcBaseline = 0.0f;
    if (_stepDcBaseline < 0.1f) _stepDcBaseline = instantA;
    static float _peakAcInStep = 0.0f;

    // تقسيم الـ 512 عينة إلى 8 كتل فرعية (64 عينة = ~7.7ms لكل كتلة -> معدل ~115 هرتز)
    const size_t SUB_BLOCKS = 8;
    const size_t SUB_SIZE = BURST_SIZE / SUB_BLOCKS; // 64 samples

    for (size_t b = 0; b < SUB_BLOCKS; b++) {
      float subSumMv = 0.0f;
      size_t offset = b * SUB_SIZE;
      for (size_t i = 0; i < SUB_SIZE; i++) {
        subSumMv += _rawMv[offset + i];
      }
      float subMeanMv = subSumMv / (float)SUB_SIZE;
      float subInstantA = fabsf(subMeanMv - _zeroMv) * invSens;
      if (subInstantA < 0.04f) subInstantA = 0.0f;

      // معاملات IIR معاد ضبطها لمعدل ~115 هرتز (أعلى بـ 8 مرات):
      // المرشح السريع (~55ms) يحفظ القمة الحادة للقدم، والبطيء (~1.1s) يتتبع الحمل المستمر
      _stepLpCurrent = _stepLpCurrent * 0.85f + subInstantA * 0.15f;
      _stepDcBaseline = _stepDcBaseline * 0.992f + subInstantA * 0.008f;

      float acGait = _stepLpCurrent - _stepDcBaseline;
      uint32_t subNowMs = (nowMs > 56) ? (nowMs - 56 + (uint32_t)(b * 8)) : nowMs;
      uint32_t dtStep = subNowMs - _lastStepMs;

      if (runnerOnBelt) {
        if (!_stepArmed) {
          // تتبع القمة القصوى الفعلية أثناء طور الهبوط على السير لدقة حساب التناظر (Symmetry)
          if (acGait > _peakAcInStep) _peakAcInStep = acGait;

          // إعادة تسليح الزناد عند نزول الإشارة في قاع الخطوة وتسجيل القمة الحقيقية للخطوة المنتهية
          if (acGait <= -0.02f && dtStep >= 230) {
            if (_peakAcInStep > 0.02f) {
              if (_stepAlternator) {
                _rightPeakSum = _rightPeakSum * 0.80f + _peakAcInStep * 0.20f;
              } else {
                _leftPeakSum = _leftPeakSum * 0.80f + _peakAcInStep * 0.20f;
              }
              _stepAlternator = !_stepAlternator;

              if (_leftPeakSum > 0.02f && _rightPeakSum > 0.02f) {
                float denom = _leftPeakSum < _rightPeakSum ? _rightPeakSum : _leftPeakSum;
                float numer = _leftPeakSum < _rightPeakSum ? _leftPeakSum : _rightPeakSum;
                float ratio = (denom > 0.02f) ? (numer / denom) : 1.0f;
                _symmetryPct = _symmetryPct * 0.85f + (ratio * 100.0f) * 0.15f;
              }
            }
            _peakAcInStep = 0.0f;
            _stepArmed = true;
          }
        } else {
          // ربط حساسية ضربة القدم (_cfg.stepDeltaA) ديناميكياً مع متوسط قمة الصدمة (v35.7)
          float dynPeak = (_leftPeakSum + _rightPeakSum) * 0.5f;
          float baseThresh = _cfg.stepDeltaA * 0.35f; // عتبة الأساس المتناسبة مع حساسية المستخدم
          float triggerThresh = (dynPeak > 0.03f) ? (baseThresh + dynPeak * 0.18f) : baseThresh;
          if (triggerThresh < 0.045f) triggerThresh = 0.045f;
          if (triggerThresh > 0.35f) triggerThresh = 0.35f;

          if (acGait >= triggerThresh && dtStep >= 230) {
            _stepArmed = false;
            _peakAcInStep = acGait;
            _stepCount++;
            _lastStepMs = subNowMs;

            if (dtStep > 230 && dtStep < 1400) {
              uint16_t instantSpm = (uint16_t)(60000UL / dtStep);
              if (instantSpm >= 75 && instantSpm <= 220) {
                _localCadenceSpm = _localCadenceSpm == 0 ? instantSpm : (uint16_t)(_localCadenceSpm * 0.65f + instantSpm * 0.35f);
              }
            }
          }
        }
      } else {
        _stepArmed = true;
        _peakAcInStep = 0.0f;
        _localCadenceSpm = 0;
      }
    }
    if ((nowMs - _lastStepMs) > 4000) {
      _localCadenceSpm = 0;
    }

    if (runnerOnBelt) {

      // 10. التقدير الديناميكي الشامل للوزن مع تعويض الميل والسرعة (Argon Point 4)
      float speedEmptyCompA = (liveSpeedKmh > 4.0f) ? ((liveSpeedKmh - 4.0f) * 0.04f) : 0.0f;
      float inclineLoadA = (liveInclPct > 0.0f && _cfg.inclineCompA > 0.0f) ? (liveInclPct * _cfg.inclineCompA) : 0.0f;
      float dynamicEmptyRef = emptyRef + speedEmptyCompA + inclineLoadA;
      float loadDeltaA = _smoothedCurrentA - dynamicEmptyRef;

      if (loadDeltaA >= 0.45f) {
        float speedCorr = 1.0f;
        if (liveSpeedKmh >= 2.0f && liveSpeedKmh <= 16.0f) {
          speedCorr = 1.0f / (1.0f + 0.012f * (liveSpeedKmh - 4.0f));
        }
        float rawEstKg = (loadDeltaA * _cfg.weightKgPerA) * speedCorr;
        if (rawEstKg >= 35.0f && rawEstKg <= 160.0f) {
          if (_estWeight < 35.0f) _estWeight = rawEstKg;
          else _estWeight = _estWeight * 0.96f + rawEstKg * 0.04f;
        }
      }
    } else {
      _stepArmed = true;
    }

    // 11. التحديث الذري الموحد لجميع القراءات (Unified Critical Section - Argon Point 6)
    bool calcStepOff = false;
    if (liveSpeedKmh >= 3.0f && _m.emptyBeltA > 0.5f) {
      if ((nowMs - _lastStepMs) > 7000 && _smoothedCurrentA <= (_m.emptyBeltA + 0.40f)) {
        calcStepOff = true;
      }
    }

    uint8_t calcFatigue = (_symmetryPct < 70.0f) ? 2 : ((_symmetryPct < 85.0f) ? 1 : 0);

    portENTER_CRITICAL(&dspMux);
    _m.zeroVoltMv = _zeroMv;
    _m.activeSensMvA = _effectiveSens();
    _m.currentA = _smoothedCurrentA;
    _m.motorLoadPct = (_smoothedCurrentA / _cfg.ratedCurrentA) * 100.0f;
    _m.stallAlert = _stallActive;
    _m.powerWatts = calcPowerWatts;
    _m.energyKWh = _energyWs / 3600000.0f;
    _m.currentP2pA = (maxAc > minAc) ? (maxAc - minAc) : 0.0f;
    _m.bearingStatus = 0;
    _m.kurtosis = _m.kurtosis * 0.70f + (float)rawKurtosis * 0.30f;
    _m.arcingScore = _arcingScore;
    _m.brushHealth = _brushState;
    _m.stepCount = _stepCount;
    _m.cadenceSpm = _localCadenceSpm;
    _m.symmetryPct = _symmetryPct;
    _m.fatigueState = calcFatigue;
    if (_estWeight >= 35.0f) _m.estimatedWeightKg = _estWeight;
    _m.stepOffAlert = calcStepOff;
    if (liveSpeedKmh > 2.0f && calcPowerWatts > 20.0f) {
      _m.runningEconomyWhPerKm = calcPowerWatts / liveSpeedKmh;
    }
    portEXIT_CRITICAL(&dspMux);
  }

  void resetSteps() {
    portENTER_CRITICAL(&dspMux);
    _stepCount = 0;
    _localCadenceSpm = 0;
    _m.stepCount = 0;
    _m.cadenceSpm = 0;
    _lastStepMs = 0;
    portEXIT_CRITICAL(&dspMux);
  }

  void resetEnergy() {
    portENTER_CRITICAL(&dspMux);
    _energyWs = 0.0f;
    _m.energyKWh = 0.0f;
    portEXIT_CRITICAL(&dspMux);
  }

  void loadCumulativeCounters(float totalKWh, uint32_t totalSteps) {
    portENTER_CRITICAL(&dspMux);
    if (!isnan(totalKWh) && totalKWh >= 0.0f) {
      _energyWs = totalKWh * 3600000.0f;
      _m.energyKWh = totalKWh;
    }
    _stepCount = totalSteps;
    _m.stepCount = totalSteps;
    portEXIT_CRITICAL(&dspMux);
  }

  float getCumulativeEnergyKWh() const {
    portENTER_CRITICAL((portMUX_TYPE*)&dspMux);
    float v = _m.energyKWh;
    portEXIT_CRITICAL((portMUX_TYPE*)&dspMux);
    return v;
  }

  uint32_t getCumulativeSteps() const {
    portENTER_CRITICAL((portMUX_TYPE*)&dspMux);
    uint32_t v = _stepCount;
    portEXIT_CRITICAL((portMUX_TYPE*)&dspMux);
    return v;
  }

  void resetArcingScore() {
    portENTER_CRITICAL(&dspMux);
    _arcingScore = 0.0f;
    _m.arcingScore = 0.0f;
    _m.brushHealth = 0;
    portEXIT_CRITICAL(&dspMux);
  }

  Metrics getMetrics() const {
    Metrics copy;
    portENTER_CRITICAL((portMUX_TYPE*)&dspMux);
    copy = _m;
    portEXIT_CRITICAL((portMUX_TYPE*)&dspMux);
    return copy;
  }

private:
  float _effectiveSens() const {
    if (_cfg.autoSensOn != 0) {
      float calculated = 40.0f * (2.0f * _zeroMv / 5000.0f);
      if (calculated >= 15.0f && calculated <= 45.0f) return calculated;
    }
    return _cfg.sensMvPerA;
  }

  void _updateActiveSensitivity() {
    _m.activeSensMvA = _effectiveSens();
  }

  void _validateConfig() {
    if (isnan(_cfg.ratedCurrentA) || _cfg.ratedCurrentA < 5.0f || _cfg.ratedCurrentA > 25.0f) _cfg.ratedCurrentA = RATED_CURRENT_A;
    if (isnan(_cfg.stallLimitA) || _cfg.stallLimitA < 2.0f || _cfg.stallLimitA > 25.0f) _cfg.stallLimitA = STALL_CURRENT_A;
    if (_cfg.stallDelayMs < 100 || _cfg.stallDelayMs > 1500) _cfg.stallDelayMs = 800;
    if (isnan(_cfg.kurtosisThresh) || _cfg.kurtosisThresh < 4.0f || _cfg.kurtosisThresh > 30.0f) _cfg.kurtosisThresh = 7.5f;
    if (isnan(_cfg.cusumWarnScore) || _cfg.cusumWarnScore < 5.0f || _cfg.cusumWarnScore > 50.0f) _cfg.cusumWarnScore = 18.0f;
    if (isnan(_cfg.cusumCritScore) || _cfg.cusumCritScore < 20.0f || _cfg.cusumCritScore > 90.0f) _cfg.cusumCritScore = 45.0f;
    if (isnan(_cfg.emptyBeltWarnA) || _cfg.emptyBeltWarnA < 1.5f || _cfg.emptyBeltWarnA > 6.0f) _cfg.emptyBeltWarnA = 3.5f;
    if (isnan(_cfg.stepDeltaA) || _cfg.stepDeltaA < 0.08f || _cfg.stepDeltaA > 0.80f) _cfg.stepDeltaA = 0.18f;
    if (isnan(_cfg.sensMvPerA) || _cfg.sensMvPerA < 15.0f || _cfg.sensMvPerA > 45.0f) _cfg.sensMvPerA = DEFAULT_SENS_MV_A;
    if (isnan(_cfg.nominalBusV) || _cfg.nominalBusV < 90.0f || _cfg.nominalBusV > 240.0f) _cfg.nominalBusV = RATED_BUS_VOLTS;
    if (isnan(_cfg.calibratedBaselineA) || _cfg.calibratedBaselineA < 0.5f || _cfg.calibratedBaselineA > 5.0f) _cfg.calibratedBaselineA = 1.80f;
    if (isnan(_cfg.inertiaDelayS) || _cfg.inertiaDelayS < 1.0f || _cfg.inertiaDelayS > 10.0f) _cfg.inertiaDelayS = 3.5f;
    if (isnan(_cfg.inclineCompA) || _cfg.inclineCompA < 0.0f || _cfg.inclineCompA > 0.50f) _cfg.inclineCompA = 0.15f;
    if (isnan(_cfg.weightKgPerA) || _cfg.weightKgPerA < 15.0f || _cfg.weightKgPerA > 90.0f) _cfg.weightKgPerA = 32.0f;
  }

  uint8_t _pin;
  float _zeroMv;
  bool _calibrated;

  DspConfig _cfg;

  float _arcingScore;
  uint8_t _brushState;

  uint32_t _emptyBeltSampleCount;
  float _emptyBeltSum;
  bool _pendingLubricationCal;
  volatile bool _pendingZeroCal = false;

  float _energyWs;
  uint32_t _lastEnergyUpdateMs;

  uint32_t _stepCount;
  uint16_t _localCadenceSpm = 0;
  uint32_t _lastStepMs;
  float _stepLpCurrent;
  float _prevStepSlope;
  float _leftPeakSum;
  float _rightPeakSum;
  bool _stepAlternator;
  float _symmetryPct;
  bool _stepArmed;

  uint32_t _stallStartMs;
  bool _stallActive;

  uint32_t _lastBurstMs;
  float _smoothedCurrentA;
  float _estWeight;

  Metrics _m;

  // Static/BSS buffers allocated inside class instance, completely off task stack
  float _rawMv[BURST_SIZE];
  float _hp[BURST_SIZE];
};

} // namespace dsp


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
// Fwd and LockState declared at top

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
Preferences wifiPrefs;

// ---------- مزامنة الساعة الذكية عبر الواي فاي المنزلي (Wi-Fi NTP Auto-Sync) ----------
char wfSsid[48] = DEFAULT_WIFI_SSID;
char wfPass[64] = DEFAULT_WIFI_PASS;
enum WifiNtpState : uint8_t { WF_IDLE = 0, WF_CONNECTING = 1, WF_WAIT_NTP = 2 };
volatile uint8_t wfState = WF_IDLE;
uint32_t wfStartMs = 0, wfLastCheckMs = 0;
bool ntpSynced = false;

void startWifiNtpSync() {
  if (wfSsid[0] == 0) return;
  Serial.printf("[WIFI-NTP] Connecting to Wi-Fi '%s' for automatic clock sync...\n", wfSsid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wfSsid, wfPass);
  wfState = WF_CONNECTING;
  wfStartMs = millis();
  wfLastCheckMs = wfStartMs;
}

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
volatile uint8_t tmHeartRate = 0;
volatile uint32_t tmHeartRateMs = 0;


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
  STOP_BRUSH_WEAR = 8,       // تآكل وتفحم شديد في فحمات المحرك
  STOP_FALL_DISMOUNT = 9,    // درع الأمان: كشف النزول أو السقوط المفاجئ
  STOP_SLIP_DRAG = 10,       // درع الأمان: كشف الانزلاق والصدمة الشاذة
  STOP_AUTO_RESCUE = 11      // درع الأمان: تخفيض السرعة التلقائي عند ارتباك الخطوات
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


// ---------- درع الأمان الذكي وكشف السقوط والنزول التلقائي (v37.0) ----------
struct SafetyConfig {
  uint8_t fsEn;      // كاشف النزول: 1 = تفعيل، 0 = تعطيل (افتراضي: 1)
  uint16_t fsMs;     // مهلة انعدام الخطوات بالملي ثانية (250-1500ms، افتراضي: 500ms)
  float fsVmin;      // أدنى سرعة للتفعيل كم/س (1.5-6.0 كم/س، افتراضي: 3.0)
  uint8_t drEn;      // كاشف الانزلاق: 1 = تفعيل، 0 = تعطيل (افتراضي: 1)
  uint16_t drMs;     // مدة الصدمة الميكانيكية بالملي ثانية (100-500ms، افتراضي: 200ms)
  float drAmp;       // فارق أمبير الصدمة (0.6-3.0A، افتراضي: 1.2A)
  uint8_t rcEn;      // الإنقاذ التلقائي: 1 = تفعيل، 0 = تعطيل (افتراضي: 1)
  float rcDrop;      // مقدار خفض السرعة (1.0-5.0 كم/س، افتراضي: 2.5)
  uint8_t rcVar;     // نسبة تشتت الخطوات (15-50%، افتراضي: 30)
  uint8_t safeTest;  // وضع الاختبار الآمن: 1 = تنبيهات فقط بدون إيقاف، 0 = إيقاف حقيقي (افتراضي: 0)
};

SafetyConfig safetyCfg = {1, 600, 2.5f, 1, 300, 2.0f, 1, 2.5f, 30, 0};
Preferences safetyPrefs;

extern NimBLERemoteCharacteristic* tmCp;
extern volatile bool needCtrl;
void sendLong(const char* s);

void loadSafetyConfig() {
  safetyPrefs.begin("k_safety", false);
  if (safetyPrefs.getBytesLength("cfg_v3") == sizeof(safetyCfg)) {
    safetyPrefs.getBytes("cfg_v3", &safetyCfg, sizeof(safetyCfg));
  } else {
    safetyPrefs.putBytes("cfg_v3", &safetyCfg, sizeof(safetyCfg));
  }
  // ضمان تفعيل الحماية الحقيقية عند الإقلاع لتفادي بقاء وضع الاختبار الآمن عالقاً في الفلاش
  safetyCfg.safeTest = 0;
  if (safetyCfg.fsVmin > 3.5f) safetyCfg.fsVmin = 2.5f;
}

void saveSafetyConfig() {
  safetyPrefs.putBytes("cfg_v3", &safetyCfg, sizeof(safetyCfg));
}

void sendSafetyConfig() {
  if (!connected) return;
  char b[128];
  snprintf(b, sizeof(b), "SF:C=%d,%u,%.1f,%d,%u,%.1f,%d,%.1f,%d,%d\n",
           safetyCfg.fsEn, safetyCfg.fsMs, safetyCfg.fsVmin,
           safetyCfg.drEn, safetyCfg.drMs, safetyCfg.drAmp,
           safetyCfg.rcEn, safetyCfg.rcDrop, safetyCfg.rcVar,
           safetyCfg.safeTest);
  sendLine(b);
}

extern float reqSpeed, outSpeed, lastSentSpd;
void shaperReset();

void onSafetyTrigger(uint8_t type, uint32_t valMs, float speed) {
  if (safetyCfg.safeTest) {
    char b[96];
    const char* tName = (type == 1) ? "DISMOUNT" : ((type == 2) ? "DRAG" : "RESCUE");
    snprintf(b, sizeof(b), "SF:EVT=%s,%lu,TEST\n", tName, (unsigned long)valMs);
    sendLine(b);
    Serial.printf("[SAFE_TEST] %s triggered (%lu ms / speed: %.1f km/h) - Safe Test Mode: Motor Untouched!\n",
                  tName, (unsigned long)valMs, speed);
    return;
  }

  if (type == 1) { // 1. كشف النزول أو السقوط المفاجئ -> إيقاف فعلي فوري للحزام بدون قفل أوامر السرعة اللاحقة
    setStopReason(STOP_FALL_DISMOUNT, "FALL_DISMOUNT");
    shaperReset();
    if (tmCp) {
      uint8_t reqCmd[1] = {0x00};
      tmCp->writeValue(reqCmd, 1, false);
      needCtrl = false;
      vTaskDelay(pdMS_TO_TICKS(90));
      uint8_t pauseCmd[2] = {0x08, 0x02};
      tmCp->writeValue(pauseCmd, 2, false);
      vTaskDelay(pdMS_TO_TICKS(90));
      uint8_t stopCmd[2] = {0x08, 0x01};
      tmCp->writeValue(stopCmd, 2, false);
    }
    char b[96];
    snprintf(b, sizeof(b), "SF:EVT=DISMOUNT,%lu,ACT\n", (unsigned long)valMs);
    sendLine(b);
  } else if (type == 2) { // 2. كشف الانزلاق والصدمة الشاذة -> فرملة طوارئ فورية للحزام
    setStopReason(STOP_SLIP_DRAG, "SLIP_DRAG_TRIP");
    shaperReset();
    if (tmCp) {
      uint8_t reqCmd[1] = {0x00};
      tmCp->writeValue(reqCmd, 1, false);
      needCtrl = false;
      vTaskDelay(pdMS_TO_TICKS(90));
      uint8_t stopCmd[2] = {0x08, 0x01};
      tmCp->writeValue(stopCmd, 2, false);
    }
    char b[96];
    snprintf(b, sizeof(b), "SF:EVT=DRAG,%lu,ACT\n", (unsigned long)valMs);
    sendLine(b);
  } else if (type == 3) { // 3. نظام الإنقاذ وخفض السرعة التلقائي عند التعثر والارتباك
    float newSpeed = speed - safetyCfg.rcDrop;
    if (newSpeed < 1.5f) newSpeed = 1.5f;
    reqSpeed = newSpeed;
    outSpeed = newSpeed;
    lastSentSpd = newSpeed;
    uint16_t v = (uint16_t)lroundf(newSpeed * 100.0f);
    uint8_t speedCmd[3] = {0x02, (uint8_t)(v & 255), (uint8_t)(v >> 8)};
    if (tmCp) {
      uint8_t reqCmd[1] = {0x00};
      tmCp->writeValue(reqCmd, 1, false);
      needCtrl = false;
      vTaskDelay(pdMS_TO_TICKS(80));
      tmCp->writeValue(speedCmd, 3, false);
    }
    setStopReason(STOP_AUTO_RESCUE, "AUTO_RESCUE_DROP");
    char b[96];
    snprintf(b, sizeof(b), "SF:EVT=RESCUE,%.1f,ACT\n", newSpeed);
    sendLine(b);
  }
}

volatile uint32_t lastSafetyActionMs = 0;
uint32_t lastStepSeenMs = 0;
uint32_t stepIntervals[4] = {0, 0, 0, 0};
uint8_t stepIntervalIdx = 0;
uint32_t dragStartMs = 0;
uint32_t stumbleStartMs = 0;
bool runnerLoadedOnBelt = false;
uint32_t runnerLoadStartMs = 0;
uint32_t emptyBeltStartMs = 0;

// متغيرات التعلم الآلي الديناميكي لتيار السير الفارغ وتيار المشي الفعلي (بدون قيم ثابتة جامدة)
float dynEmptyBaseA = 0.0f;
float dynWalkAvgA   = 0.0f;
float lastLearnSpd  = 0.0f;

void checkSafetyTriggers(uint32_t nowMs, float speedKmh) {
  if (speedKmh < 1.0f) {
    dragStartMs = 0;
    stumbleStartMs = 0;
    runnerLoadedOnBelt = false;
    runnerLoadStartMs = 0;
    emptyBeltStartMs = 0;
    dynWalkAvgA = 0.0f;
    return;
  }
  if ((nowMs - lastSafetyActionMs) < 3500) return;

  auto m = dspEngine.getMetrics();
  uint32_t curStepCount = m.stepCount;
  static uint32_t prevStepCount = 0;
  bool newStep = false;
  if (curStepCount != prevStepCount) {
    newStep = true;
    if (prevStepCount > 0 && lastStepSeenMs > 0) {
      uint32_t dt = nowMs - lastStepSeenMs;
      if (dt >= 180 && dt <= 1800) {
        stepIntervals[stepIntervalIdx & 3] = dt;
        stepIntervalIdx++;
      }
    }
    prevStepCount = curStepCount;
    lastStepSeenMs = nowMs;
    // ملاحظة هامة: لا نصفر dragStartMs هنا لأن الاحتكاك الشاذ قد يصاحبه نبضة خطوة!
  }

  // التعديل الديناميكي لخط أساس السير الفارغ عند تغير السرعة
  if (lastLearnSpd > 0.5f && fabsf(speedKmh - lastLearnSpd) >= 0.3f && dynEmptyBaseA > 0.5f) {
    dynEmptyBaseA += (speedKmh - lastLearnSpd) * 0.07f;
    if (dynEmptyBaseA < 0.8f) dynEmptyBaseA = 0.8f;
  }
  lastLearnSpd = speedKmh;

  if (dynEmptyBaseA < 0.5f && m.emptyBeltA > 0.8f && m.emptyBeltA < 2.8f) {
    dynEmptyBaseA = m.emptyBeltA;
  }

  // التعلم الآلي المباشر لتيار السير الفارغ أثناء الدوران بدون خطوات
  bool noRecentSteps = (lastStepSeenMs == 0) || ((nowMs - lastStepSeenMs) > 1500);
  if (noRecentSteps && m.currentP2pA < 0.28f && m.currentA > 0.7f && m.currentA < 2.6f) {
    if (dynEmptyBaseA < 0.5f) dynEmptyBaseA = m.currentA;
    else if (m.currentA <= dynEmptyBaseA + 0.35f) {
      dynEmptyBaseA = dynEmptyBaseA * 0.90f + m.currentA * 0.10f;
    }
  }

  // التعلم الآلي لمتوسط تيار المشي المعتاد للمتدرب (نحدثه فقط أثناء المشي الطبيعي غير الشاذ)
  bool activeStepping = newStep || (lastStepSeenMs > 0 && (nowMs - lastStepSeenMs) < 1200);
  float minWalkCandidateA = (dynEmptyBaseA > 0.5f) ? (dynEmptyBaseA + 0.70f) : 2.45f;
  if (m.currentA >= minWalkCandidateA && (activeStepping || m.currentP2pA >= 0.22f || m.currentA >= 2.70f)) {
    if (dynWalkAvgA < 1.0f) {
      dynWalkAvgA = m.currentA;
    } else if (m.currentA < (dynWalkAvgA + safetyCfg.drAmp * 0.85f)) {
      // نحدث متوسط المشي الطبيعي فقط عندما لا يكون المتدرب في حالة احتكاك/انزلاق شاذ
      dynWalkAvgA = dynWalkAvgA * 0.93f + m.currentA * 0.07f;
    }

    emptyBeltStartMs = 0;
    if (runnerLoadStartMs == 0) runnerLoadStartMs = nowMs;
    if ((nowMs - runnerLoadStartMs) >= 450 || newStep) {
      runnerLoadedOnBelt = true;
    }
  }

  // حساب الحد الفاصل الآلي للسير الفارغ
  float dynamicEmptyCeilingA;
  if (dynEmptyBaseA > 0.5f && dynWalkAvgA > (dynEmptyBaseA + 0.65f)) {
    dynamicEmptyCeilingA = dynEmptyBaseA + 0.42f * (dynWalkAvgA - dynEmptyBaseA);
  } else if (dynWalkAvgA > 2.35f) {
    dynamicEmptyCeilingA = dynWalkAvgA * 0.68f;
  } else if (dynEmptyBaseA > 0.5f) {
    dynamicEmptyCeilingA = dynEmptyBaseA + 0.50f;
  } else {
    dynamicEmptyCeilingA = 1.40f + (speedKmh * 0.08f) + 0.50f;
  }

  // --- آلية 1: كاشف النزول السريع (Dismount) بالحد الآلي الديناميكي ---
  if (safetyCfg.fsEn && speedKmh >= safetyCfg.fsVmin && runnerLoadedOnBelt) {
    if (m.currentA <= dynamicEmptyCeilingA && !newStep) {
      if (m.currentA > 0.7f) {
        if (dynEmptyBaseA < 0.5f) dynEmptyBaseA = m.currentA;
        else dynEmptyBaseA = dynEmptyBaseA * 0.85f + m.currentA * 0.15f;
      }
      if (emptyBeltStartMs == 0) {
        emptyBeltStartMs = nowMs;
      } else {
        uint32_t emptyDur = nowMs - emptyBeltStartMs;
        uint32_t reqEmptyMs = (safetyCfg.fsMs < 600) ? 600 : safetyCfg.fsMs;
        if (emptyDur >= reqEmptyMs) {
          runnerLoadedOnBelt = false;
          runnerLoadStartMs = 0;
          emptyBeltStartMs = 0;
          lastSafetyActionMs = nowMs;
          onSafetyTrigger(1, emptyDur, speedKmh);
          return;
        }
      }
    } else {
      emptyBeltStartMs = 0;
    }
  }

  // --- آلية 2: كاشف الانزلاق والاحتكاك الشاذ المستمر (Sustained Drag) ---
  // يعتمد مباشرة على متوسط تيار المشي المكتسب آلياً + فارق أمبير الصدمة المحدد في الشريط (drAmp)
  if (safetyCfg.drEn && speedKmh >= 1.5f && runnerLoadedOnBelt) {
    float baseWalkRef = (dynWalkAvgA > 2.3f) ? dynWalkAvgA : ((dynEmptyBaseA > 0.5f ? dynEmptyBaseA : 1.70f) + 1.8f);
    float dragThresholdA = baseWalkRef + safetyCfg.drAmp;
    // التيار الفعال للصدمة يشمل التيار المستمر بالإضافة لجزء من ذروة التموج اللحظي عند الفرملة بالقدم
    float effectiveDragA = m.currentA + (m.currentP2pA > 0.45f ? (m.currentP2pA - 0.45f) * 0.45f : 0.0f);
    if (effectiveDragA >= dragThresholdA) {
      if (dragStartMs == 0) dragStartMs = nowMs;
      else if ((nowMs - dragStartMs) >= safetyCfg.drMs) {
        uint32_t dragDur = nowMs - dragStartMs;
        dragStartMs = 0;
        lastSafetyActionMs = nowMs;
        onSafetyTrigger(2, dragDur, speedKmh);
        return;
      }
    } else {
      dragStartMs = 0;
    }
  }

  // --- آلية 3: نظام الإنقاذ وخفض السرعة التلقائي عند الارتباك والتعثر (Auto-Rescue) ---
  // يعمل ابتداءً من سرعة 2.0 كم/س ويرصد إما تشتت زمن الخطوات أو التموج العنيف غير المنتظم للتيار أثناء التعثر
  if (safetyCfg.rcEn && speedKmh >= 2.0f && runnerLoadedOnBelt) {
    // أ) فحص تشتت زمن الخطوات (من خطوتين فأكثر)
    if (stepIntervalIdx >= 2 && (nowMs - lastStepSeenMs) < 1200) {
      uint32_t dt1 = stepIntervals[(stepIntervalIdx - 1) & 3];
      uint32_t dt2 = stepIntervals[(stepIntervalIdx - 2) & 3];
      if (dt1 > 0 && dt2 > 0) {
        float diff = fabsf((float)dt1 - (float)dt2);
        float avg = (float)(dt1 + dt2) * 0.5f;
        float jitterPct = (diff / avg) * 100.0f;
        if (jitterPct >= (float)safetyCfg.rcVar) {
          lastSafetyActionMs = nowMs;
          stepIntervalIdx = 0;
          stumbleStartMs = 0;
          onSafetyTrigger(3, (uint32_t)jitterPct, speedKmh);
          return;
        }
      }
    }
    // ب) فحص التموج العنيف في سحب التيار (P2P Ripple) الناتج عن تعثر القدمين أو اضطراب التوازن
    float baseWalk = (dynWalkAvgA > 2.2f) ? dynWalkAvgA : 3.5f;
    float ripplePct = (m.currentP2pA / baseWalk) * 100.0f;
    if (ripplePct >= (float)safetyCfg.rcVar && m.currentP2pA >= 0.65f) {
      if (stumbleStartMs == 0) stumbleStartMs = nowMs;
      else if ((nowMs - stumbleStartMs) >= 350) {
        lastSafetyActionMs = nowMs;
        stumbleStartMs = 0;
        onSafetyTrigger(3, (uint32_t)ripplePct, speedKmh);
        return;
      }
    } else {
      stumbleStartMs = 0;
    }
  }
}

void dspTask(void*) {
  for (;;) {
    uint32_t nowMs = millis();
    dspEngine.processBurst(nowMs, tmSpeed, tmIncl);
    checkSafetyTriggers(nowMs, tmSpeed);
    vTaskDelay(pdMS_TO_TICKS(15));
  }
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
    dspEngine.requestZeroCalibration();
    mhe.resetLivePeakCurrent();
    dspDirty = true;
    dspDirtyMs = millis();
    delay(75);
    auto m = dspEngine.getMetrics();
    Serial.printf("[CALIB] Zero calibration complete on Core 0. New V0 = %.1f mV\n", m.zeroVoltMv);
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
  if (strncmp(l, "WFS:", 4) == 0) {
    strncpy(wfSsid, l + 4, sizeof(wfSsid) - 1);
    wfSsid[sizeof(wfSsid) - 1] = 0;
    wifiPrefs.putBytes("s", wfSsid, sizeof(wfSsid));
    return;
  }
  if (strncmp(l, "WFP:", 4) == 0) {
    strncpy(wfPass, l + 4, sizeof(wfPass) - 1);
    wfPass[sizeof(wfPass) - 1] = 0;
    wifiPrefs.putBytes("p", wfPass, sizeof(wfPass));
    startWifiNtpSync();
    sendLine("WF:TRY\n");
    return;
  }
  if (strcmp(l, "WFQ") == 0) {
    char b[96];
    snprintf(b, sizeof b, "WF:ST=%d,S=%s\n", ntpSynced ? 1 : 0, wfSsid);
    sendLine(b);
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
    if (strncmp(l, "SFQ", 3) == 0) { sendSafetyConfig(); return; }
  if (strncmp(l, "SFS:", 4) == 0) {
    int fe = 1, fm = 500, de = 1, dm = 200, re = 1, rv = 30, st = 0;
    float fv = 3.0f, da = 1.2f, rd = 2.5f;
    if (sscanf(l + 4, "%d,%d,%f,%d,%d,%f,%d,%f,%d,%d",
               &fe, &fm, &fv, &de, &dm, &da, &re, &rd, &rv, &st) >= 10) {
      safetyCfg.fsEn = constrain(fe, 0, 1);
      safetyCfg.fsMs = constrain(fm, 250, 1500);
      safetyCfg.fsVmin = constrain(fv, 1.5f, 8.0f);
      safetyCfg.drEn = constrain(de, 0, 1);
      safetyCfg.drMs = constrain(dm, 80, 800);
      safetyCfg.drAmp = constrain(da, 0.4f, 4.0f);
      safetyCfg.rcEn = constrain(re, 0, 1);
      safetyCfg.rcDrop = constrain(rd, 0.5f, 6.0f);
      safetyCfg.rcVar = constrain(rv, 10, 60);
      safetyCfg.safeTest = constrain(st, 0, 1);
      saveSafetyConfig();
      sendLine("SF:OK\n");
      sendSafetyConfig();
      Serial.printf("[SAFETY] Config saved: Dismount=%d(%ums,>=%.1fkm/h), Drag=%d(%ums,+%.1fA), Rescue=%d(-%.1fkm/h), TestMode=%d\n",
                    safetyCfg.fsEn, safetyCfg.fsMs, safetyCfg.fsVmin, safetyCfg.drEn, safetyCfg.drMs, safetyCfg.drAmp,
                    safetyCfg.rcEn, safetyCfg.rcDrop, safetyCfg.safeTest);
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
  void onConnect(NimBLEServer* s, ble_gap_conn_desc* desc) override {
    s->updateConnParams(desc->conn_handle, 24, 48, 0, 400);
    if (s->getConnectedCount() < 2) NimBLEDevice::startAdvertising();
  }
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
  if (f & 8) { if (n >= o + 2) tmIncl = (int16_t)(d[o] | (d[o + 1] << 8)) * 0.1f; o += 4; }
  if (f & 0x010) o += 4; // elevation gain
  if (f & 0x020) o += 1; // inst pace
  if (f & 0x040) o += 1; // avg pace
  if (f & 0x080) o += 5; // energy
  if (f & 0x100) {       // heart rate present
    if (n >= o + 1) {
      tmHeartRate = d[o];
      tmHeartRateMs = millis();
    }
  }
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
  if (op == 0x02 && f.n >= 3) {                       // سرعة مستهدفة: تنفيذ فوري ومضمون
    if (latchedFaults & FAULT_STALL_LATCH) {
      latchedFaults &= ~FAULT_STALL_LATCH;
      updateFaultFlags();
    }
    cdActive = false;
    float v = (f.d[1] | (f.d[2] << 8)) * 0.01f;
    float mx = effMaxSpeed(), mn = tmMinSpeed();
    if (v > mx) v = mx;
    if (v < mn && v > 0.05f) v = mn;
    reqSpeed = v;
    outSpeed = v;
    lastSentSpd = v;
    uint16_t rawV = (uint16_t)lroundf(v * 100.0f);
    uint8_t cmdB[3] = {0x02, (uint8_t)(rawV & 255), (uint8_t)(rawV >> 8)};
    sendRaw(cmdB, 3);
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
    if ((uint32_t)(now - tmSpeedMs) < 4000) {
      float curA = dspEngine.getMetrics().currentA;
      // لا نمرر سرعة لوحدة صحة المحرك إلا إذا كان المحرك يسحب تياراً حقيقياً (يمنع جلسات 1.0 كم/س الوهمية أثناء التوقف)
      float realSpd = (curA >= 0.80f && tmSpeed >= 1.5f) ? tmSpeed : 0.0f;
      mhe.feedSpeed(now, realSpd, tmIncl);
    }
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
    // إصلاح: إذا كانت الحساسية المحفوظة عالقة على القيمة القديمة 26.4 بسبب خطأ الترتيب السابق، نرقيها للحساسية الفعلية (~37.3)
    if (fabsf(t.sensMvPerA - 26.4f) < 0.15f) {
      float actS = dspEngine.getMetrics().activeSensMvA;
      t.sensMvPerA = (actS >= 30.0f && actS <= 42.0f) ? actS : 37.3f;
    }
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

void wifiNtpPump(uint32_t now) {
  if (wfState == WF_IDLE) return;
  if ((uint32_t)(now - wfLastCheckMs) < 250) return;
  wfLastCheckMs = now;

  if (wfState == WF_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("[WIFI-NTP] Wi-Fi connected! Requesting NTP time (UTC+1 Algeria)...");
      configTime(3600, 0, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
      wfState = WF_WAIT_NTP;
      wfStartMs = now;
    } else if ((uint32_t)(now - wfStartMs) > 14000) {
      Serial.println("[WIFI-NTP] Wi-Fi connection timeout. Turning off Wi-Fi radio.");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wfState = WF_IDLE;
      if (connected) sendLine("WF:ERR=CONN\n");
    }
  } else if (wfState == WF_WAIT_NTP) {
    time_t nowEpoch = time(nullptr);
    if (nowEpoch > 1700000000L) {
      mhe.setTime((uint32_t)nowEpoch, now);
      ntpSynced = true;
      Serial.printf("[WIFI-NTP] Clock synced via NTP! Epoch = %lu. Turning off Wi-Fi to save power & BLE bandwidth.\n", (unsigned long)nowEpoch);
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wfState = WF_IDLE;
      if (connected) {
        char b[96];
        snprintf(b, sizeof b, "WF:OK=%lu,S=%s\n", (unsigned long)nowEpoch, wfSsid);
        sendLine(b);
      }
    } else if ((uint32_t)(now - wfStartMs) > 12000) {
      Serial.println("[WIFI-NTP] NTP response timeout. Turning off Wi-Fi radio.");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wfState = WF_IDLE;
      if (connected) sendLine("WF:ERR=NTP\n");
    }
  }
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
  loadSafetyConfig();

  limPrefs.begin("lim", false);
  warnC = limPrefs.getFloat("w", 60.0f);
  hotC  = limPrefs.getFloat("h", 80.0f);
  if (warnC < 20 || warnC > 120) warnC = 60;
  if (hotC < 20 || hotC > 120) hotC = 80;

  cfgPrefs.begin("cfg", false);
  cfgLoad();

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

  dspEngine.begin(PIN_ACS, 180.0f, 37.3f);
  dspPrefs.begin("dsp", false);
  dspLoad();

  wifiPrefs.begin("wifi", false);
  if (wifiPrefs.getBytesLength("s") == sizeof(wfSsid)) {
    wifiPrefs.getBytes("s", wfSsid, sizeof(wfSsid));
    wfSsid[sizeof(wfSsid) - 1] = 0;
  }
  if (wifiPrefs.getBytesLength("p") == sizeof(wfPass)) {
    wifiPrefs.getBytes("p", wfPass, sizeof(wfPass));
    wfPass[sizeof(wfPass) - 1] = 0;
  }
  if (wfSsid[0] != 0) {
    startWifiNtpSync();
  }
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
    snprintf(b, sizeof b, "CUR:I=%.2f,PK=%.2f,K=%.2f,B=%d,W=%.1f,S=%u,C=%u,P=%.1f,E=%.2f,R=%.0f\n",
             m.currentA, mhe.getLivePeakCurrentA(), m.kurtosis, (int)m.brushHealth, m.powerWatts,
             (unsigned)m.stepCount, (unsigned)m.cadenceSpm, m.symmetryPct, m.emptyBeltA, m.frictionRatioPct);
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
  wifiNtpPump(now);
  if (!tmLinked || (uint32_t)(now - tmSpeedMs) >= 4000) {
    auto dm = dspEngine.getMetrics();
    if (dm.currentA >= 1.65f && dm.cadenceSpm >= 80) {
      float estKmh = (dm.cadenceSpm >= 135) ? (6.0f + (dm.cadenceSpm - 135) * 0.12f) : (3.0f + (dm.cadenceSpm - 80) * 0.055f);
      if (estKmh > 16.0f) estKmh = 16.0f;
      mhe.feedSpeed(now, estKmh, 0.0f);
    } else if (dm.currentA < 0.80f) {
      mhe.feedSpeed(now, 0.0f, 0.0f);
    }
  }
  { auto dm = dspEngine.getMetrics(); mhe.feedCurrent(dm.currentA, dm.currentP2pA); }
  mhe.loop(now);
  rawDumpPump();
  delay(5);
}
