// MotorHealth.h - Motor temperature health analysis that runs ON the ESP32.
// Version: Argon-Spark Unified Edition (120-min Cold Soak & Ambient Tracking)
#pragma once
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
static const int START_N = 4;        // 20 s of running starts a session
static const int STOP_N = 24;        // 120 s stopped ends it
static const int PLATEAU_MIN = 60;   // a stable plateau must last 5 min
static const int LEAD_N = 6;         // skip the first 30 s of the plateau
static const int WIN_MAX = 72;       // analyse at most 6 min of it
static const int SKIP_N = 12;        // ignore the first minute of the run
static const int NSB = 8;            // speed bins: <2,2-4,4-6,...,>=14 km/h
static const int NIB = 3;            // incline bins: <1.5, <5, >=5 %
static const int NBASE = 5;          // sessions that make the baseline
static const int NREC = 3;           // recent sessions compared with it
static const int HIST = 40;          // sessions kept
static const int16_t NONE = -32768;
static const uint32_t COLD_AFTER_S = 120 * 60;  // 120 min (2 hours) ensures true thermal equilibrium of heavy PMDC rotor

struct Session {          // 20 bytes
  uint32_t epoch;         // start time, 0 = unknown
  uint16_t durMin;        // running minutes
  uint8_t avg10;          // average speed x10
  uint8_t flags;          // 1 cold start, 2 heating rate valid, 4 cool-down valid, 8 incomplete
  int16_t start100;       // temperature at start x100
  int16_t peak100;        // highest temperature x100
  int16_t heat100;        // deg C / min x100
  int16_t tau10;          // minutes x10
  uint8_t sbin, ibin;     // 255 = none
  uint8_t pad[2];
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
typedef void (*SendFn)(const char*, size_t);
typedef void (*SleepFn)(uint32_t);

inline int16_t med(const int16_t* a, int n) {
  int16_t t[8];
  for (int i = 0; i < n; i++) t[i] = a[i];
  for (int i = 1; i < n; i++) { int16_t v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
  return (n & 1) ? t[n / 2] : (int16_t)((t[n / 2 - 1] + t[n / 2]) / 2);
}

inline void binAdd(Bin& b, int16_t v) {
  if (b.nBase < NBASE) { b.base[b.nBase++] = v; return; }
  b.rec[b.pos] = v;
  b.pos = (uint8_t)((b.pos + 1) % NREC);
  if (b.nRec < NREC) b.nRec++;
}

inline bool binReady(const Bin& b) { return b.nBase == NBASE && b.nRec == NREC; }

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
      if (loadFn(&tmp, sizeof tmp) && tmp.magic == 0x4D484C54u && tmp.ver == STORE_VER && tmp.size == sizeof(Store)) st = tmp;
    }
  }
  void setChunk(size_t n) { chunkSz = n < 5 ? 5 : n; }

  void setTime(uint32_t epoch, uint32_t now) { epochBase = epoch; msBase = now; timeSet = true; }
  void feedTemp(uint32_t now, float t) {
    if (isnan(t) || t < -40.f || t > 200.f) return;
    lastT = t; lastTMs = now; haveT = true;
    // Track minimum ambient reference when motor is idle
    if (stt == IDLE && (!haveAmbient || t < ambientRefT)) {
      ambientRefT = t;
      haveAmbient = true;
    }
    sampler(now);
  }
  void feedSpeed(uint32_t now, float kmh, float incl) {
    if (isnan(kmh) || kmh < 0.f || kmh > 40.f) return;
    lastS = kmh; lastI = isnan(incl) ? 0.f : incl; lastSMs = now; haveS = true;
  }
  void loop(uint32_t now) {
    char c[40];
    while (popCommand(c, sizeof c)) command(c, now);
    sampler(now);
    pump(now);
  }

  bool pushCommand(const char* s) {
    if (!s || s[0] != 'M') return false;
    uint8_t h = inHead, n = (uint8_t)((h + 1) % IN_Q);
    if (n == inTail) return false;
    strncpy(inbox[h], s, sizeof inbox[h] - 1);
    inbox[h][sizeof inbox[h] - 1] = 0;
    inHead = n;
    return true;
  }

  Status status() const {
    Status r; memset(&r, 0, sizeof r);
    int level = 0;
    for (int i = 0; i < NSB * NIB; i++) {
      const Bin& b = st.bins[i];
      if (b.nBase > 0 && b.nBase < NBASE) r.learning++;
      if (!binReady(b)) continue;
      r.judged++;
      int j = judge(med(b.base, NBASE), med(b.rec, NREC), 15, 30, 8, 15);
      if (j > level) level = j;
      if (j >= 2) r.reasons |= 1;
    }
    if (binReady(st.tau)) {
      r.judged++;
      int j = judge(med(st.tau.base, NBASE), med(st.tau.rec, NREC), 25, 50, 30, 60);
      if (j > level) level = j;
      if (j >= 2) r.reasons |= 2;
    }
    r.level = r.judged ? (uint8_t)(level < 1 ? 1 : level) : 0;
    return r;
  }
  int state() const { return (int)stt; }

  void command(const char* line, uint32_t now) {
    if (!line || line[0] != 'M') return;
    char c = line[1];
    const char* a = line + 2;
    char b[96];
    if (c == 'S') {
      Status s = status();
      snprintf(b, sizeof b, "MH:S,%u,%u,%u,%lu,%lu,%d,%d,%u,%u",
               s.level, s.reasons, (unsigned)st.count,
               (unsigned long)(st.totalSec / 60), (unsigned long)((st.totalSec - st.maintSec) / 60),
               (int)stt, timeSet ? 1 : 0, s.judged, s.learning);
      out(b);
      out("MH:E,S");
    } else if (c == 'L') {
      int n = atoi(a); if (n <= 0) n = 10; if (n > 20) n = 20;
      int have = st.count < HIST ? st.count : HIST;
      if (n > have) n = have;
      for (int k = 0; k < n; k++) {
        int idx = ((int)st.head - 1 - k + HIST * 2) % HIST;
        const Session& s = st.hist[idx];
        snprintf(b, sizeof b, "MH:L,%d,%lu,%u,%u,%d,%d,%d,%d,%u,%u,%u", k, (unsigned long)s.epoch, s.durMin, s.avg10,
                 s.start100, s.peak100, s.heat100, s.tau10, s.sbin, s.ibin, s.flags);
        out(b);
      }
      out("MH:E,L");
    } else if (c == 'B') {
      for (int i = 0; i < NSB * NIB; i++) {
        const Bin& x = st.bins[i];
        if (!x.nBase) continue;
        snprintf(b, sizeof b, "MH:B,%d,%d,%u,%d,%u,%d", i / NIB, i % NIB, x.nBase,
                 x.nBase == NBASE ? med(x.base, NBASE) : NONE, x.nRec, x.nRec == NREC ? med(x.rec, NREC) : NONE);
        out(b);
      }
      if (st.tau.nBase) {
        snprintf(b, sizeof b, "MH:B,255,0,%u,%d,%u,%d", st.tau.nBase,
                 st.tau.nBase == NBASE ? med(st.tau.base, NBASE) : NONE, st.tau.nRec,
                 st.tau.nRec == NREC ? med(st.tau.rec, NREC) : NONE);
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
  float lastT = 0, lastS = 0, lastI = 0; uint32_t lastTMs = 0, lastSMs = 0; bool haveT = false, haveS = false;
  float ambientRefT = 25.0f; bool haveAmbient = false;
  bool sampStarted = false; uint32_t nextSample = 0;

  int stt = IDLE;
  int runCnt = 0, stopCnt = 0, unkCnt = 0;
  int earlyN = 0; bool earlyBad = false;
  int16_t eT[EARLY_N]; uint8_t eS[EARLY_N]; int16_t eI[EARLY_N];
  int coolN = 0; int16_t cT[COOL_N];
  uint32_t runTicks = 0; float sumSpeed = 0; float startT = 0, peakT = 0, lastRunT = 0;
  uint32_t runStartEpoch = 0; bool cold = false; bool incomplete = false;
  bool haveEndMs = false; uint32_t lastEndMs = 0;
  int curIdx = -1;

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
    if ((int32_t)(now - nextSample) > 60000) nextSample = now;
    while ((int32_t)(now - nextSample) >= 0) { tick(nextSample); nextSample += SAMPLE_MS; }
  }

  bool computeCold(uint32_t now) const {
    if (haveEndMs) return (uint32_t)(now - lastEndMs) / 1000u >= COLD_AFTER_S;
    if (timeSet && st.lastEnd > 0) { uint32_t e = epochAt(now); return e > st.lastEnd && e - st.lastEnd >= COLD_AFTER_S; }
    float minCold = haveAmbient ? ambientRefT : 1e9f;
    bool any = haveAmbient;
    int have = st.count < HIST ? st.count : HIST;
    for (int i = 0; i < have; i++) {
      const Session& s = st.hist[i];
      if ((s.flags & 1) && s.start100 != NONE) { float v = s.start100 / 100.f; if (v < minCold) minCold = v; any = true; }
    }
    return !any || startT <= minCold + 2.5f;
  }

  void tick(uint32_t tm) {
    bool tOk = haveT && (int32_t)(tm - lastTMs) <= 15000;
    bool sOk = haveS && (int32_t)(tm - lastSMs) <= 15000;
    float t = lastT, s = sOk ? lastS : -1.f, inc = lastI;
    bool running = sOk && s >= 0.5f;

    if (stt == IDLE || stt == COOL) {
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

    if (tOk && t > peakT) peakT = t;
    if (running) {
      stopCnt = 0; coolN = 0; unkCnt = 0;
      if (tOk) { lastRunT = t; }
      runTicks++; sumSpeed += s;
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
    int ii = (int)lroundf(inc * 10.f);
    pushEarlyRaw((int16_t)lroundf(t * 100.f), (uint8_t)v, (int16_t)ii);
  }
  void pushEarlyRaw(int16_t t, uint8_t s, int16_t i) {
    if (earlyN >= EARLY_N) return;
    eT[earlyN] = t; eS[earlyN] = s; eI[earlyN] = i; earlyN++;
  }

  void beginRun(uint32_t tm) {
    stt = RUN; stopCnt = 0; unkCnt = 0; coolN = 0; incomplete = false;
    runTicks = 0; sumSpeed = 0;
    startT = (earlyN && eT[0] != NONE) ? eT[0] / 100.f : lastT;
    lastRunT = (earlyN && eT[earlyN - 1] != NONE) ? eT[earlyN - 1] / 100.f : lastT;
    peakT = (startT > -50.0f) ? startT : ((lastT > -50.0f) ? lastT : 25.0f);
    for (int i = 0; i < earlyN; i++) { runTicks++; sumSpeed += eS[i] / 10.f; if (eT[i] != NONE && eT[i] / 100.f > peakT) peakT = eT[i] / 100.f; }
    uint32_t t0 = tm - (uint32_t)(earlyN ? earlyN - 1 : 0) * SAMPLE_MS;
    runStartEpoch = epochAt(t0);
    cold = computeCold(t0);
    if (cold) { ambientRefT = startT; haveAmbient = true; }
    earlyBad = false; curIdx = -1;
  }

  bool heatWindow(float& slope, int& sb, int& ib) const {
    if (earlyBad) return false;
    int i = SKIP_N;
    while (i < earlyN) {
      if (eT[i] == NONE || eS[i] < 20) { i++; continue; }
      int minS = eS[i], maxS = eS[i], minI = eI[i], maxI = eI[i], j = i;
      while (j + 1 < earlyN && eT[j + 1] != NONE && eS[j + 1] >= 20) {
        int ns = eS[j + 1], ni = eI[j + 1];
        int a = minS < ns ? minS : ns, b = maxS > ns ? maxS : ns, c = minI < ni ? minI : ni, d = maxI > ni ? maxI : ni;
        if (b - a > 10 || d - c > 20) break;
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
        slope = (float)((wn * sxy - sx * sy) / den);
        ms /= wn; mi /= wn;
        sb = (int)(ms / 2.0); if (sb < 0) sb = 0; if (sb >= NSB) sb = NSB - 1;
        ib = mi < 1.5 ? 0 : (mi < 5.0 ? 1 : 2);
        return true;
      }
      i = j + 1;
    }
    return false;
  }

  void endRun(uint32_t tm, bool incompleteEnd) {
    Session r; memset(&r, 0, sizeof r);
    r.epoch = runStartEpoch;
    uint32_t sec = runTicks * (SAMPLE_MS / 1000);
    r.durMin = (uint16_t)(sec / 60 > 65535 ? 65535 : sec / 60);
    int a = runTicks ? (int)lroundf(sumSpeed / runTicks * 10.f) : 0; r.avg10 = (uint8_t)(a > 255 ? 255 : a);
    r.start100 = (int16_t)lroundf(startT * 100.f); r.peak100 = (int16_t)lroundf(peakT * 100.f);
    r.heat100 = NONE; r.tau10 = NONE; r.sbin = 255; r.ibin = 255;
    if (cold) r.flags |= 1;
    if (incompleteEnd || incomplete) r.flags |= 8;
    float slope; int sb, ib;
    if (heatWindow(slope, sb, ib)) {
      int h = (int)lroundf(slope * 100.f);
      r.heat100 = (int16_t)h; r.sbin = (uint8_t)sb; r.ibin = (uint8_t)ib; r.flags |= 2;
      if (cold && h > 0) binAdd(st.bins[sb * NIB + ib], (int16_t)h);
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
      int span = coolN >= 120 ? 120 : 60;
      float refAmb = (haveAmbient && ambientRefT < startT) ? ambientRefT : startT;
      float excess = lastRunT - refAmb;
      float drop = lastRunT - cT[span - 1] / 100.f;
      if ((r.flags & 1) && excess >= 5.f) {
        float frac = drop / excess;
        if (frac >= 0.03f && frac <= 0.95f) {
          float tau = -(span * SAMPLE_MS / 60000.f) / logf(1.f - frac);
          int t10 = (int)lroundf(tau * 10.f); if (t10 > 3000) t10 = 3000;
          r.tau10 = (int16_t)t10; r.flags |= 4;
          binAdd(st.tau, (int16_t)t10);
        }
      }
      save();
    }
    stt = IDLE; coolN = 0; curIdx = -1;
  }
};

}  // namespace mh
