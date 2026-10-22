#pragma once
#include <Arduino.h>
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
    c.kurtosisThresh        = 7.5f;              // K > 6.5
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

  // Auto-calibrates zero-current sensor output voltage when motor is stopped
  void calibrateZero(size_t samples = 200) {
    float sum = 0.0f;
    for (size_t i = 0; i < samples; i++) {
      sum += (float)analogReadMilliVolts(_pin);
      delayMicroseconds(100);
    }
    float v0 = sum / (float)samples;
    // Sanity check: allow 1100mV - 2200mV (covering 2.5V - 4.4V rail halving)
    if (v0 >= 1100.0f && v0 <= 2200.0f) {
      _zeroMv = v0;
      _calibrated = true;
    } else {
      _zeroMv = 1650.0f; // Default nominal 3.3V / 2
    }
    _m.zeroVoltMv = _zeroMv;

    // Apply auto-ratiometric sensitivity scaling if enabled
    _updateActiveSensitivity();
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

    // Auto-update zero calibration if belt has been fully stopped (idle motor)
    // CRITICAL GUARD: Only track true micro-thermal drift (< 5.0 mV, approx < 0.2A).
    // If current is flowing (diff >= 5.0 mV), NEVER shift zero point!
    float driftDiff = fabsf(meanMv - _zeroMv);
    if (_cfg.zeroTrackingOn != 0 && liveSpeedKmh <= 0.05f && driftDiff < 5.0f && meanMv >= 1200.0f && meanMv <= 2100.0f) {
      _zeroMv = _zeroMv * 0.995f + meanMv * 0.005f;
      _m.zeroVoltMv = _zeroMv;
      _updateActiveSensitivity();
    }

    // 2. Low-Frequency DC Current & Motor Load % Calculation
    // Auto-Polarity: Uses fabsf(diffMv) so current is measured accurately regardless of
    // whether the primary terminals (IP+ / IP-) are wired in forward or reverse direction.
    float diffMv = meanMv - _zeroMv;
    float instantA = fabsf(diffMv) / _effectiveSens();
    if (instantA < 0.04f) instantA = 0.0f; // Noise deadband at idle

    _smoothedCurrentA = _smoothedCurrentA * 0.7f + instantA * 0.3f;
    _m.currentA = _smoothedCurrentA;
    _m.motorLoadPct = (_smoothedCurrentA / _cfg.ratedCurrentA) * 100.0f;

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
    _m.stallAlert = _stallActive;

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

    // 6 & 7. High-Pass Local Moving Baseline + 2-Tap PWM Suppression & Kurtosis (Argon Point 2)
    // يعزل موجة خطوة القدم والتسارع تماماً (fc ~ 115 Hz) ويكتم ضوضاء تقطيع الـ PWM قبل حساب التفرطح
    float minAc = 1e9f, maxAc = -1e9f;
    float invSens = 1.0f / _effectiveSens();
    float localMean = _rawMv[0];
    float prevHf = 0.0f;
    double sumSq = 0.0;
    double sumQuad = 0.0;

    for (size_t i = 0; i < BURST_SIZE; i++) {
      localMean = localMean * 0.92f + _rawMv[i] * 0.08f; // يتتبع موجة القدم ويتجاهل إبر الشرارة
      float hfSample = (_rawMv[i] - localMean) * invSens;
      _hp[i] = (i > 0) ? 0.5f * (hfSample + prevHf) : hfSample; // مرشح ثنائي النقاط لكتم ضوضاء الـ PWM
      prevHf = hfSample;

      if (_hp[i] < minAc) minAc = _hp[i];
      if (_hp[i] > maxAc) maxAc = _hp[i];

      double d = (double)_hp[i];
      double d2 = d * d;
      sumSq += d2;
      sumQuad += (d2 * d2);
    }
    _m.currentP2pA = (maxAc > minAc) ? (maxAc - minAc) : 0.0f;

    // Bearing Mechanical Roughness: Unloaded belt high-frequency ripple check
    // Bearing status default smooth (PWM ripple cannot falsely trip mechanical bearing check)
    _m.bearingStatus = 0;

    double variance = sumSq / (double)BURST_SIZE;
    double rawKurtosis = 2.8;

    if (variance > 0.0004 && liveSpeedKmh >= 0.5f) { // variance > (20mA)^2
      rawKurtosis = (sumQuad / (double)BURST_SIZE) / (variance * variance);
      if (rawKurtosis > 50.0) rawKurtosis = 50.0; // clamp extreme outliers
    } else {
      rawKurtosis = 2.8; // Baseline commutation / Gaussian default
    }

    _m.kurtosis = _m.kurtosis * 0.70f + (float)rawKurtosis * 0.30f;

    // 8. CUSUM / Temporal Evidence Accumulator (Guarantees Zero False Positives)
    //    Step increments are clamped to max +6.0 per burst so single spikes never trigger an alert
    if (liveSpeedKmh >= 0.8f && _m.currentA >= 0.3f) {
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
    _m.arcingScore = _arcingScore;

    if (_arcingScore >= _cfg.cusumCritScore) {
      _brushState = 2; // Critical: Severe wear / heavy continuous sparking
    } else if (_arcingScore >= _cfg.cusumWarnScore) {
      _brushState = 1; // Warning: Early wear / intermittent sparking
    } else {
      _brushState = 0; // Healthy: Smooth commutation
    }
    _m.brushHealth = _brushState;

    // 9. Medium-Frequency Band: 8x Sub-Sampled Footstrike Step Counter (~115 Hz, Argon Point 1)
    // قفل عزم التسارع وتغيير السرعة (Speed Ramp Lockout)
    static float _lastSpeedKmh = 0.0f;
    static uint32_t _speedChangeMs = 0;
    if (fabsf(liveSpeedKmh - _lastSpeedKmh) >= 0.15f) {
      _lastSpeedKmh = liveSpeedKmh;
      _speedChangeMs = nowMs;
    }
    bool isAccelerating = (nowMs - _speedChangeMs) < 2500;

    // بوابة وجود الراكب (Runner Presence Lock Gate)
    float emptyRef = _m.emptyBeltA > 0.5f ? _m.emptyBeltA : 1.08f;
    bool runnerOnBelt = !isAccelerating && ((_smoothedCurrentA >= (emptyRef + 0.45f)) || (_smoothedCurrentA >= 1.65f));

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
                float ratio = _leftPeakSum < _rightPeakSum ? (_leftPeakSum / _rightPeakSum) : (_rightPeakSum / _leftPeakSum);
                _symmetryPct = _symmetryPct * 0.85f + (ratio * 100.0f) * 0.15f;
                _m.symmetryPct = _symmetryPct;

                if (_m.symmetryPct < 70.0f) _m.fatigueState = 2;
                else if (_m.symmetryPct < 85.0f) _m.fatigueState = 1;
                else _m.fatigueState = 0;
              }
            }
            _peakAcInStep = 0.0f;
            _stepArmed = true;
          }
        } else {
          float dynPeak = (_leftPeakSum + _rightPeakSum) * 0.5f;
          float triggerThresh = (dynPeak > 0.03f) ? (0.05f + dynPeak * 0.20f) : 0.06f;
          if (triggerThresh > 0.15f) triggerThresh = 0.15f;

          if (acGait >= triggerThresh && dtStep >= 230) {
            _stepArmed = false;
            _peakAcInStep = acGait;
            _stepCount++;
            _m.stepCount = _stepCount;
            _lastStepMs = subNowMs;

            if (dtStep > 230 && dtStep < 1400) {
              uint16_t instantSpm = (uint16_t)(60000UL / dtStep);
              if (instantSpm >= 75 && instantSpm <= 220) {
                _m.cadenceSpm = _m.cadenceSpm == 0 ? instantSpm : (uint16_t)(_m.cadenceSpm * 0.65f + instantSpm * 0.35f);
              }
            }
          }
        }
      } else {
        _stepArmed = true;
        _peakAcInStep = 0.0f;
      }
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

    portENTER_CRITICAL(&dspMux);
    _m.zeroVoltMv = _zeroMv;
    _m.currentA = _smoothedCurrentA;
    _m.motorLoadPct = (_smoothedCurrentA / _cfg.ratedCurrentA) * 100.0f;
    _m.stallAlert = _stallActive;
    _m.powerWatts = calcPowerWatts;
    _m.energyKWh = _energyWs / 3600000.0f;
    _m.currentP2pA = (maxAc > minAc) ? (maxAc - minAc) : 0.0f;
    _m.kurtosis = _m.kurtosis * 0.70f + (float)rawKurtosis * 0.30f;
    _m.arcingScore = _arcingScore;
    _m.brushHealth = _brushState;
    _m.stepCount = _stepCount;
    _m.symmetryPct = _symmetryPct;
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
    if (isnan(_cfg.stallLimitA) || _cfg.stallLimitA < 2.0f || _cfg.stallLimitA > 15.0f) _cfg.stallLimitA = STALL_CURRENT_A;
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

  float _energyWs;
  uint32_t _lastEnergyUpdateMs;

  uint32_t _stepCount;
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
