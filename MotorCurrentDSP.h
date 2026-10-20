#pragma once
#include <Arduino.h>
#include <math.h>

/**
 * MotorCurrentDSP.h - High-Speed Multi-Band Digital Signal Processing (DSP) Engine
 * Designed for ESP32-S3 + Allegro ACS758LCB-050B (50A Hall-Effect Current Sensor)
 * Version: Argon-Spark Unified Industrial Edition
 */

namespace dsp {

static portMUX_TYPE dspMux = portMUX_INITIALIZER_UNLOCKED;

struct DspConfig {
  float ratedCurrentA;        // Continuous Rated Current (Nameplate: 10.0A, Range: 5.0 - 25.0A)
  float stallLimitA;          // Stall Trip Limit (Default 150%: 15.0A, Range: 8.0 - 25.0A)
  uint16_t stallDelayMs;      // Trip Persistence Duration (Default: 800ms, Range: 100 - 1500ms)
  float kurtosisThresh;       // Arcing Detection Kurtosis Threshold (Default: 7.5, Range: 4.0 - 30.0)
  float cusumWarnScore;       // CUSUM Warning Accumulator Score (Default: 18.0, Range: 5.0 - 50.0)
  float cusumCritScore;       // CUSUM Critical Accumulator Score (Default: 45.0, Range: 20.0 - 90.0)
  float emptyBeltWarnA;       // Unloaded Belt Lubrication Warn Limit (Default: 3.5A, Range: 1.5 - 6.0A)
  float stepDeltaA;           // Footstrike Peak Threshold Delta (Default: 0.18A, Range: 0.08 - 0.80A)
  float sensMvPerA;           // ACS758 Sensitivity on 3.3V (Default: 26.4 mV/A, Range: 15.0 - 45.0)
  float nominalBusV;          // Nominal DC Bus Voltage (Default: 180.0V, Range: 90.0 - 240.0V)
  float calibratedBaselineA;  // Clean empty belt baseline captured right after lubrication (Default: 1.80A)
  uint8_t autoSensOn;         // 1 = Auto-calculate from zero voltage, 0 = Manual sensMvPerA (Default: 1)
  float inertiaDelayS;        // Delay before lubrication baseline measurement (Default: 3.5s, Range: 1.0 - 10.0s)
  float inclineCompA;         // Incline current compensation per 1% grade at 75kg (Default: 0.15A/%, Range: 0.0 - 0.50A/%)
  uint8_t zeroTrackingOn;     // 1 = Dynamic thermal zero tracking at idle, 0 = Fixed zero (Default: 1)
  float weightKgPerA;         // Universal Body Mass Calibration Constant (Default: 32.0 kg/A, Range: 15.0 - 90.0 kg/A)
};

struct Metrics {
  float currentA;              // Average DC current in Amperes
  float currentP2pA;           // High-frequency peak-to-peak AC ripple current in Amperes
  float motorLoadPct;          // Instantaneous motor electrical load (relative to ratedCurrentA)
  float kurtosis;              // 4th standardized moment (healthy ~2.8-3.5, arcing > kurtosisThresh)
  float arcingScore;           // CUSUM cumulative confidence score (0 to 100)
  uint8_t brushHealth;         // 0 = Healthy/Smooth, 1 = Warning (Early Arcing), 2 = Critical (Severe Wear)
  float emptyBeltA;            // Baseline empty belt current in first 5s of run (lubrication metric)
  float calibratedBaselineA;   // Pristine baseline captured right after lubrication
  float frictionRatioPct;      // Friction ratio relative to pristine lubrication (100% = fresh oil)
  bool lubricationAlert;       // True if emptyBeltA exceeds emptyBeltWarnA
  float powerWatts;            // Instantaneous electrical power in Watts (V_eff * I_avg)
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
  static const size_t SUB_BLOCKS = 8;
  static const size_t SUB_SIZE   = BURST_SIZE / SUB_BLOCKS; // 64 samples per sub-block

  // Motor Specifications from Nameplate (Kettler Dortmund S2 - Cat. No. 3030200146)
  static constexpr float RATED_BUS_VOLTS   = 180.0f;
  static constexpr float RATED_CURRENT_A   = 10.0f;
  static constexpr float RATED_HORSEPOWER  = 2.5f;
  static constexpr float STALL_CURRENT_A   = 15.0f;
  static constexpr float DEFAULT_SENS_MV_A = 26.4f;
  static constexpr float ARMATURE_R_OHM    = 1.5f;   // Typical winding + brush resistance for 2.5HP 180V PMDC

  static DspConfig getDefaultConfig() {
    DspConfig c;
    c.ratedCurrentA         = RATED_CURRENT_A;   // 10.0f
    c.stallLimitA           = STALL_CURRENT_A;   // 15.0f
    c.stallDelayMs          = 800;               // 800 ms
    c.kurtosisThresh        = 7.5f;              // Optimized threshold after 2-tap PWM & High-Pass filter
    c.cusumWarnScore        = 18.0f;             // Score >= 18.0 -> Warning
    c.cusumCritScore        = 45.0f;             // Score >= 45.0 -> Critical
    c.emptyBeltWarnA        = 3.5f;              // Empty belt > 3.5A -> Needs silicone lubrication
    c.stepDeltaA            = 0.18f;             // Sensitive footstrike load pulse
    c.sensMvPerA            = DEFAULT_SENS_MV_A; // 26.4 mV/A
    c.nominalBusV           = RATED_BUS_VOLTS;   // 180.0 V
    c.calibratedBaselineA   = 1.80f;             // Clean pristine baseline after lubrication
    c.autoSensOn            = 1;                 // Default: Auto-calculate from zero voltage
    c.inertiaDelayS         = 3.5f;              // 3.5 seconds start delay to bypass acceleration
    c.inclineCompA          = 0.15f;             // 0.15 A per 1% grade (normalized to 75kg user)
    c.zeroTrackingOn        = 1;                 // Default: Auto-track thermal zero at idle
    c.weightKgPerA          = 32.0f;             // Calibrated universal constant: 32.0 kg/A
    return c;
  }

  MotorCurrentDSP()
    : _pin(5), _zeroMv(1650.0f), _calibrated(false),
      _arcingScore(0.0f), _brushState(0),
      _emptyBeltSampleCount(0), _emptyBeltSum(0.0f),
      _pendingLubricationCal(false), _windowContaminated(false),
      _energyWs(0.0f), _lastEnergyUpdateMs(0),
      _stepCount(0), _lastStepMs(0),
      _stepLpCurrent(0.0f), _stepDcBaseline(0.0f), _peakAcInStep(0.0f),
      _leftPeakSum(0.0f), _rightPeakSum(0.0f),
      _stepAlternator(false), _symmetryPct(100.0f), _stepArmed(true),
      _stallStartMs(0), _stallActive(false),
      _lastBurstMs(0), _smoothedCurrentA(0.0f),
      _estWeight(75.0f)
  {
    _cfg = getDefaultConfig();
    memset(&_m, 0, sizeof(_m));
    memset(_rawMv, 0, sizeof(_rawMv));
    memset(_hp, 0, sizeof(_hp));
    _m.kurtosis = 3.0f;
    _m.brushHealth = 0;
    _m.symmetryPct = 100.0f;
    _m.calibratedBaselineA = _cfg.calibratedBaselineA;
    _m.emptyBeltA = _cfg.calibratedBaselineA;
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
    analogSetPinAttenuation(_pin, ADC_11db);
    calibrateZero(300);
    _lastEnergyUpdateMs = millis();
  }

  void onMaintenanceLubricated() {
    portENTER_CRITICAL(&dspMux);
    _pendingLubricationCal = true;
    _emptyBeltSampleCount = 0;
    _emptyBeltSum = 0.0f;
    _windowContaminated = false;
    portEXIT_CRITICAL(&dspMux);
  }

  bool isLubricationCalPending() const {
    return _pendingLubricationCal;
  }

  void calibrateZero(size_t samples = 200) {
    float sum = 0.0f;
    for (size_t i = 0; i < samples; i++) {
      sum += (float)analogReadMilliVolts(_pin);
      delayMicroseconds(100);
    }
    float v0 = sum / (float)samples;
    portENTER_CRITICAL(&dspMux);
    if (v0 >= 1100.0f && v0 <= 2200.0f) {
      _zeroMv = v0;
      _calibrated = true;
    } else {
      _zeroMv = 1650.0f;
    }
    _m.zeroVoltMv = _zeroMv;
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
    portENTER_CRITICAL(&dspMux);
    _cfg = getDefaultConfig();
    _updateActiveSensitivity();
    portEXIT_CRITICAL(&dspMux);
  }

  void processBurst(uint32_t nowMs, float liveSpeedKmh, float liveInclPct = 0.0f) {
    if (nowMs - _lastBurstMs < 45) return;
    _lastBurstMs = nowMs;

    // 1. High-speed burst capture (~8.33 kHz nominal)
    float sumMv = 0.0f;
    for (size_t i = 0; i < BURST_SIZE; i++) {
      _rawMv[i] = (float)analogReadMilliVolts(_pin);
      sumMv += _rawMv[i];
      delayMicroseconds(110); // Compensated for ADC execution time (~10us) -> exact 120us period
    }

    float meanMv = sumMv / (float)BURST_SIZE;

    // Thread-safe snapshot of configuration & state
    portENTER_CRITICAL(&dspMux);
    DspConfig cfg = _cfg;
    float curZeroMv = _zeroMv;
    portEXIT_CRITICAL(&dspMux);

    // Thermal Zero-Current Tracking at Idle
    float driftDiff = fabsf(meanMv - curZeroMv);
    if (cfg.zeroTrackingOn != 0 && liveSpeedKmh <= 0.05f && driftDiff < 4.5f && meanMv >= 1200.0f && meanMv <= 2100.0f) {
      curZeroMv = curZeroMv * 0.995f + meanMv * 0.005f;
      portENTER_CRITICAL(&dspMux);
      _zeroMv = curZeroMv;
      _updateActiveSensitivity();
      portEXIT_CRITICAL(&dspMux);
    }

    float effSens = _effectiveSens();
    float invSens = 1.0f / effSens;

    // 2. Low-Frequency DC Current & Motor Load
    float diffMv = meanMv - curZeroMv;
    float instantA = fabsf(diffMv) * invSens;
    if (instantA < 0.04f) instantA = 0.0f;

    _smoothedCurrentA = _smoothedCurrentA * 0.70f + instantA * 0.30f;
    float calcLoadPct = (_smoothedCurrentA / cfg.ratedCurrentA) * 100.0f;

    // 3. Electronic Stall / Overload Protection
    if (instantA >= cfg.stallLimitA) {
      if (_stallStartMs == 0) _stallStartMs = nowMs;
      else if (nowMs - _stallStartMs >= (uint32_t)cfg.stallDelayMs) {
        _stallActive = true;
      }
    } else {
      _stallStartMs = 0;
      _stallActive = false;
    }

    // 4. Electromechanical PMDC Model for Power & Energy (Back-EMF + Ia*Ra)
    float effectiveBusV = 0.0f;
    if (liveSpeedKmh > 0.05f) {
      float ratedBackEmf = cfg.nominalBusV - (cfg.ratedCurrentA * ARMATURE_R_OHM);
      if (ratedBackEmf < 100.0f) ratedBackEmf = cfg.nominalBusV * 0.88f;
      float backEmfV = ratedBackEmf * (liveSpeedKmh / 20.0f);
      effectiveBusV = backEmfV + (_smoothedCurrentA * ARMATURE_R_OHM);
      if (effectiveBusV > cfg.nominalBusV) effectiveBusV = cfg.nominalBusV;
    } else if (_smoothedCurrentA > 0.45f) {
      // Unlinked treadmill fallback or motor stalled at 0 km/h
      effectiveBusV = (_smoothedCurrentA * ARMATURE_R_OHM) + (cfg.nominalBusV * 0.12f);
      if (effectiveBusV > cfg.nominalBusV) effectiveBusV = cfg.nominalBusV;
    }
    float calcPowerWatts = effectiveBusV * _smoothedCurrentA;

    float dtSec = _lastEnergyUpdateMs > 0 ? (float)(nowMs - _lastEnergyUpdateMs) / 1000.0f : 0.075f;
    _lastEnergyUpdateMs = nowMs;
    if (dtSec > 0.0f && dtSec < 5.0f && calcPowerWatts > 0.0f) {
      portENTER_CRITICAL(&dspMux);
      _energyWs += calcPowerWatts * dtSec;
      portEXIT_CRITICAL(&dspMux);
    }

    // Pre-compute 8x Sub-Blocks (64 samples each = ~7.68ms) for both Step Detection & Contamination Guard
    float subInstantA[SUB_BLOCKS];
    float minSubA = 1e9f, maxSubA = -1e9f;
    for (size_t b = 0; b < SUB_BLOCKS; b++) {
      float subSumMv = 0.0f;
      size_t offset = b * SUB_SIZE;
      for (size_t i = 0; i < SUB_SIZE; i++) {
        subSumMv += _rawMv[offset + i];
      }
      float subMeanMv = subSumMv * (1.0f / (float)SUB_SIZE);
      float sA = fabsf(subMeanMv - curZeroMv) * invSens;
      if (sA < 0.04f) sA = 0.0f;
      subInstantA[b] = sA;
      if (sA < minSubA) minSubA = sA;
      if (sA > maxSubA) maxSubA = sA;
    }
    float subSpreadA = (maxSubA > minSubA) ? (maxSubA - minSubA) : 0.0f;

    // 5. Empty-Belt Baseline Capture & Runner-Contamination Guard
    static uint32_t runStartMs = 0;
    static float prevSpeedKmh = 0.0f;
    static uint32_t stepsAtWindowStart = 0;

    if (prevSpeedKmh <= 0.05f && liveSpeedKmh > 0.5f) {
      runStartMs = nowMs;
      _emptyBeltSampleCount = 0;
      _emptyBeltSum = 0.0f;
      _windowContaminated = false;
      stepsAtWindowStart = _stepCount;
    }
    prevSpeedKmh = liveSpeedKmh;

    uint32_t startDelayMs = (uint32_t)(cfg.inertiaDelayS * 1000.0f);
    if (startDelayMs < 1000) startDelayMs = 1000;
    if (startDelayMs > 10000) startDelayMs = 10000;
    uint32_t endDelayMs = startDelayMs + 5000;

    float curEmptyBeltA = _m.emptyBeltA > 0.5f ? _m.emptyBeltA : cfg.calibratedBaselineA;
    bool curLubAlert = _m.lubricationAlert;
    uint8_t curTensionStatus = _m.beltTensionStatus;

    if (runStartMs > 0 && (nowMs - runStartMs) >= startDelayMs && (nowMs - runStartMs) <= endDelayMs) {
      if ((nowMs - runStartMs) < (startDelayMs + 90)) {
        stepsAtWindowStart = _stepCount;
      }
      float compA = (liveInclPct > 0.0f && cfg.inclineCompA > 0.0f) ? (liveInclPct * cfg.inclineCompA * 0.4f) : 0.0f;
      float netBeltCurrent = (instantA > compA) ? (instantA - compA) : instantA;

      // Contamination check: step detected, high current (>2.8A when not calibrating), or footstrike ripple (>0.22A across sub-blocks)
      bool stepDetected = (_stepCount != stepsAtWindowStart) || (subSpreadA > 0.22f);
      bool currentTooHigh = (!_pendingLubricationCal && netBeltCurrent > 2.85f) || (netBeltCurrent > 4.2f);

      if (stepDetected || currentTooHigh) {
        _windowContaminated = true;
      }

      if (!_windowContaminated) {
        _emptyBeltSum += netBeltCurrent;
        _emptyBeltSampleCount++;
        curEmptyBeltA = _emptyBeltSum / (float)_emptyBeltSampleCount;

        if (_pendingLubricationCal && _emptyBeltSampleCount >= 20) {
          if (curEmptyBeltA >= 0.75f && curEmptyBeltA <= 4.0f) {
            portENTER_CRITICAL(&dspMux);
            _cfg.calibratedBaselineA = curEmptyBeltA;
            _cfg.emptyBeltWarnA = curEmptyBeltA * 1.55f;
            cfg = _cfg;
            portEXIT_CRITICAL(&dspMux);
          }
          _pendingLubricationCal = false;
        }

        curLubAlert = (curEmptyBeltA >= cfg.emptyBeltWarnA);

        if (curEmptyBeltA > 0.5f) {
          if (curEmptyBeltA < 1.15f) curTensionStatus = 0;       // Loose
          else if (curEmptyBeltA <= 2.75f) curTensionStatus = 1; // Optimal
          else curTensionStatus = 2;                             // Over-tight
        }
      } else {
        // Fallback to clean calibrated baseline if user stood on belt during startup window
        curEmptyBeltA = cfg.calibratedBaselineA > 0.5f ? cfg.calibratedBaselineA : 1.80f;
      }
    }

    float baseRef = cfg.calibratedBaselineA > 0.5f ? cfg.calibratedBaselineA : 1.80f;
    float calcFrictionRatio = (curEmptyBeltA / baseRef) * 100.0f;

    // 6 & 7. High-Pass Local Moving Baseline + 2-Tap PWM Suppression & Kurtosis
    float minAc = 1e9f, maxAc = -1e9f;
    float localMean = _rawMv[0];
    float prevHf = 0.0f;
    double sumSq = 0.0;
    double sumQuad = 0.0;

    for (size_t i = 0; i < BURST_SIZE; i++) {
      localMean = localMean * 0.92f + _rawMv[i] * 0.08f; // Tracks mechanical foot wave, isolates commutator spikes
      float hfSample = (_rawMv[i] - localMean) * invSens;
      _hp[i] = (i > 0) ? 0.5f * (hfSample + prevHf) : hfSample; // 2-tap moving average suppresses PWM carrier
      prevHf = hfSample;

      if (_hp[i] < minAc) minAc = _hp[i];
      if (_hp[i] > maxAc) maxAc = _hp[i];

      double d = (double)_hp[i];
      double d2 = d * d;
      sumSq += d2;
      sumQuad += (d2 * d2);
    }
    float calcP2pA = (maxAc > minAc) ? (maxAc - minAc) : 0.0f;

    uint8_t curBearingStatus = _m.bearingStatus;
    if (liveSpeedKmh > 0.5f && _smoothedCurrentA < 2.8f && !_windowContaminated) {
      if (calcP2pA <= 0.45f) curBearingStatus = 0;
      else if (calcP2pA <= 0.75f) curBearingStatus = 1;
      else curBearingStatus = 2;
    }

    double variance = sumSq / (double)BURST_SIZE;
    double rawKurtosis = 3.0;
    if (variance > 0.0004 && liveSpeedKmh >= 0.5f) {
      rawKurtosis = (sumQuad / (double)BURST_SIZE) / (variance * variance);
      if (rawKurtosis > 50.0) rawKurtosis = 50.0;
    }

    // 8. CUSUM Temporal Evidence Accumulator
    if (liveSpeedKmh >= 0.8f && _smoothedCurrentA >= 0.3f) {
      if (rawKurtosis >= cfg.kurtosisThresh) {
        float inc = (float)(rawKurtosis - (cfg.kurtosisThresh - 1.5f)) * 0.8f;
        if (inc > 6.0f) inc = 6.0f;
        _arcingScore += inc;
      } else if (rawKurtosis <= (cfg.kurtosisThresh - 2.0f)) {
        _arcingScore -= 2.0f;
      }
    } else {
      _arcingScore -= 3.0f;
    }

    if (_arcingScore < 0.0f) _arcingScore = 0.0f;
    if (_arcingScore > 100.0f) _arcingScore = 100.0f;

    if (_arcingScore >= cfg.cusumCritScore) _brushState = 2;
    else if (_arcingScore >= cfg.cusumWarnScore) _brushState = 1;
    else _brushState = 0;

    // 9. Medium-Frequency Band: 8x Sub-Sampled Footstrike Step Counter (~115 Hz)
    static float lastSpeedKmh = 0.0f;
    static uint32_t speedChangeMs = 0;
    if (fabsf(liveSpeedKmh - lastSpeedKmh) >= 0.15f) {
      lastSpeedKmh = liveSpeedKmh;
      speedChangeMs = nowMs;
    }
    bool isAccelerating = (nowMs - speedChangeMs) < 2500;

    float emptyRef = curEmptyBeltA > 0.5f ? curEmptyBeltA : 1.08f;
    bool runnerOnBelt = !isAccelerating && ((_smoothedCurrentA >= (emptyRef + 0.45f)) || (_smoothedCurrentA >= 1.65f));

    uint16_t curCadence = _m.cadenceSpm;
    uint8_t curFatigue = _m.fatigueState;

    for (size_t b = 0; b < SUB_BLOCKS; b++) {
      float subA = subInstantA[b];
      if (_stepDcBaseline < 0.1f) _stepDcBaseline = subA;

      _stepLpCurrent  = _stepLpCurrent * 0.85f + subA * 0.15f;
      _stepDcBaseline = _stepDcBaseline * 0.992f + subA * 0.008f;

      float acGait = _stepLpCurrent - _stepDcBaseline;
      uint32_t subNowMs = (nowMs > 56) ? (nowMs - 56 + (uint32_t)(b * 8)) : nowMs;
      uint32_t dtStep = subNowMs - _lastStepMs;

      if (runnerOnBelt) {
        if (!_stepArmed) {
          if (acGait > _peakAcInStep) _peakAcInStep = acGait;
          if (acGait <= -0.02f && dtStep >= 230) {
            if (_peakAcInStep > 0.02f) {
              if (_stepAlternator) _rightPeakSum = _rightPeakSum * 0.80f + _peakAcInStep * 0.20f;
              else                 _leftPeakSum  = _leftPeakSum  * 0.80f + _peakAcInStep * 0.20f;
              _stepAlternator = !_stepAlternator;
              if (_leftPeakSum > 0.02f && _rightPeakSum > 0.02f) {
                float ratio = _leftPeakSum < _rightPeakSum ? (_leftPeakSum / _rightPeakSum) : (_rightPeakSum / _leftPeakSum);
                _symmetryPct = _symmetryPct * 0.85f + (ratio * 100.0f) * 0.15f;
                if (_symmetryPct < 70.0f) curFatigue = 2;
                else if (_symmetryPct < 85.0f) curFatigue = 1;
                else curFatigue = 0;
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
            portENTER_CRITICAL(&dspMux);
            _stepCount++;
            portEXIT_CRITICAL(&dspMux);
            _lastStepMs = subNowMs;
            if (dtStep > 230 && dtStep < 1400) {
              uint16_t instantSpm = (uint16_t)(60000UL / dtStep);
              if (instantSpm >= 75 && instantSpm <= 220) {
                curCadence = (curCadence == 0) ? instantSpm : (uint16_t)(curCadence * 0.65f + instantSpm * 0.35f);
              }
            }
          }
        }
      } else {
        _stepArmed = true;
        _peakAcInStep = 0.0f;
      }
    }

    // 10. Physics-Informed Dynamic Body Weight Estimator (Mass-Proportional Incline & Speed Compensation)
    if (runnerOnBelt) {
      float speedEmptyCompA = (liveSpeedKmh > 4.0f) ? ((liveSpeedKmh - 4.0f) * 0.04f) : 0.0f;
      // Scale incline current compensation proportionally to current estimated weight relative to 75kg reference
      float weightScale = (_estWeight >= 35.0f && _estWeight <= 160.0f) ? (_estWeight / 75.0f) : 1.0f;
      float inclineLoadA = (liveInclPct > 0.0f && cfg.inclineCompA > 0.0f) ? (liveInclPct * cfg.inclineCompA * weightScale) : 0.0f;
      float dynamicEmptyRef = emptyRef + speedEmptyCompA + inclineLoadA;
      float loadDeltaA = _smoothedCurrentA - dynamicEmptyRef;

      if (loadDeltaA >= 0.45f) {
        float speedCorr = 1.0f;
        if (liveSpeedKmh >= 2.0f && liveSpeedKmh <= 16.0f) {
          speedCorr = 1.0f / (1.0f + 0.012f * (liveSpeedKmh - 4.0f));
        }
        float rawEstKg = (loadDeltaA * cfg.weightKgPerA) * speedCorr;
        if (rawEstKg >= 35.0f && rawEstKg <= 160.0f) {
          if (_estWeight < 35.0f) _estWeight = rawEstKg;
          else _estWeight = _estWeight * 0.96f + rawEstKg * 0.04f;
        }
      }
    }

    // Step-Off / Unloaded Belt Detector
    bool calcStepOff = false;
    if (liveSpeedKmh >= 3.0f && curEmptyBeltA > 0.5f) {
      if ((nowMs - _lastStepMs) > 7000 && _smoothedCurrentA <= (curEmptyBeltA + 0.40f)) {
        calcStepOff = true;
      }
    }

    // Unified Atomic Commit to Metrics Struct
    portENTER_CRITICAL(&dspMux);
    _m.zeroVoltMv           = _zeroMv;
    _m.activeSensMvA        = effSens;
    _m.currentA             = _smoothedCurrentA;
    _m.motorLoadPct         = calcLoadPct;
    _m.stallAlert           = _stallActive;
    _m.powerWatts           = calcPowerWatts;
    _m.energyKWh            = _energyWs / 3600000.0f;
    _m.emptyBeltA           = curEmptyBeltA;
    _m.calibratedBaselineA  = _cfg.calibratedBaselineA;
    _m.frictionRatioPct     = calcFrictionRatio;
    _m.lubricationAlert     = curLubAlert;
    _m.beltTensionStatus    = curTensionStatus;
    _m.currentP2pA          = calcP2pA;
    _m.bearingStatus        = curBearingStatus;
    _m.kurtosis             = _m.kurtosis * 0.70f + (float)rawKurtosis * 0.30f;
    _m.arcingScore          = _arcingScore;
    _m.brushHealth          = _brushState;
    _m.stepCount            = _stepCount;
    _m.cadenceSpm           = curCadence;
    _m.symmetryPct          = _symmetryPct;
    _m.fatigueState         = curFatigue;
    if (_estWeight >= 35.0f) _m.estimatedWeightKg = _estWeight;
    _m.stepOffAlert         = calcStepOff;
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
    float val = _m.energyKWh;
    portEXIT_CRITICAL((portMUX_TYPE*)&dspMux);
    return val;
  }

  uint32_t getCumulativeSteps() const {
    portENTER_CRITICAL((portMUX_TYPE*)&dspMux);
    uint32_t val = _stepCount;
    portEXIT_CRITICAL((portMUX_TYPE*)&dspMux);
    return val;
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
    if (isnan(_cfg.stallLimitA) || _cfg.stallLimitA < 8.0f || _cfg.stallLimitA > 25.0f) _cfg.stallLimitA = STALL_CURRENT_A;
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
  bool _windowContaminated;

  float _energyWs;
  uint32_t _lastEnergyUpdateMs;

  uint32_t _stepCount;
  uint32_t _lastStepMs;
  float _stepLpCurrent;
  float _stepDcBaseline;
  float _peakAcInStep;
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

  float _rawMv[BURST_SIZE];
  float _hp[BURST_SIZE];
};

} // namespace dsp
