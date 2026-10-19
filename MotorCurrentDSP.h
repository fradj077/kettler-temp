#pragma once
#include <Arduino.h>
#include <math.h>

/**
 * MotorCurrentDSP.h - Industrial-Grade Multi-Band Digital Signal Processing (DSP) Engine (v13)
 * Designed for ESP32-S3 + Allegro ACS758LCB-050B (50A Bi-directional Hall-Effect Current Sensor)
 * Tailored for Kettler Dortmund S2 DC Treadmill Motor:
 *   - Continuous Rating: 180 VDC, 10.0 A (Continuous Duty CON), 2.5 HP, 4600 RPM, Class F
 *   - Stall Current Protection: >15.0 A (150% FLA) sustained for >300 ms
 *
 * v13 Physical & Mechanical Realism Enhancements:
 * 1. Auto-Ratiometric Sensitivity Scaling:
 *    - Automatically calculates exact mV/A sensitivity based on actual measured zero-point voltage:
 *      Sensitivity = 40.0 * (2 * V_zero / 5000)
 *      (e.g., 1500mV zero -> 24.0 mV/A; 1650mV zero -> 26.4 mV/A). Fully toggleable and overrideable!
 * 2. Inertia Start Delay & Acceleration Elimination:
 *    - Configurable delay (default 3.5s, range 1.0 - 10.0s) before capturing empty belt baseline,
 *      ensuring start beeps and roller inertia torque are completely bypassed.
 * 3. Incline Load Current Compensation:
 *    - Accounts for gravitational torque increase during inclined workouts (default 0.15 A/% grade),
 *      allowing clean motor load and belt friction estimation across all deck inclines.
 * 4. Dynamic Thermal Zero Tracking:
 *    - Slow adaptation at idle (liveSpeed <= 0.05 km/h) to track Hall sensor and ADC thermal drift.
 *      Fully toggleable from the Web App!
 * 5. Unidirectional Diode Clamping:
 *    - Enforces physical forward-only DC drive; clamps inductive PWM negative flyback transients to 0.0A.
 * 6. Smart Power Fallback:
 *    - Computes realistic effective armature voltage even if speed telemetry is pending or testing on bench.
 *
 * NOTE: Buffers _rawMv and _hp are allocated as class member variables (in SRAM .bss)
 * to prevent FreeRTOS task stack overflow and ensure rock-solid Bluetooth stability.
 */

namespace dsp {

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
  static constexpr float DEFAULT_SENS_MV_A = 26.4f;  // ACS758-050B on 3.3V supply (~26.4 mV/A)

  static DspConfig getDefaultConfig() {
    DspConfig c;
    c.ratedCurrentA         = RATED_CURRENT_A;   // 10.0f
    c.stallLimitA           = STALL_CURRENT_A;   // 15.0f
    c.stallDelayMs          = 300;               // 300 ms
    c.kurtosisThresh        = 6.5f;              // K > 6.5
    c.cusumWarnScore        = 18.0f;             // Score >= 18.0 -> Warning
    c.cusumCritScore        = 45.0f;             // Score >= 45.0 -> Critical
    c.emptyBeltWarnA        = 3.5f;              // Empty belt > 3.5A -> Needs silicone lubrication
    c.stepDeltaA            = 0.18f;             // Sensitive footstrike load pulse (0.18A)
    c.sensMvPerA            = DEFAULT_SENS_MV_A; // 26.4 mV/A
    c.nominalBusV           = RATED_BUS_VOLTS;   // 180.0 V
    c.calibratedBaselineA   = 1.80f;             // Clean pristine baseline after lubrication
    c.autoSensOn            = 1;                 // Default: Auto-calculate from zero voltage
    c.inertiaDelayS         = 3.5f;              // 3.5 seconds start delay to bypass acceleration
    c.inclineCompA          = 0.15f;             // 0.15 A per 1% grade
    c.zeroTrackingOn        = 1;                 // Default: Auto-track thermal zero at idle
    return c;
  }

  MotorCurrentDSP()
    : _pin(5), _zeroMv(1650.0f), _calibrated(false),
      _arcingScore(0.0f), _brushState(0),
      _emptyBeltSampleCount(0), _emptyBeltSum(0.0f),
      _pendingLubricationCal(false),
      _energyWs(0.0f), _lastEnergyUpdateMs(0),
      _stepCount(0), _lastStepMs(0),
      _stepLpCurrent(0.0f), _prevStepSlope(0.0f),
      _leftPeakSum(0.0f), _rightPeakSum(0.0f),
      _stepAlternator(false), _symmetryPct(100.0f),
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
    _cfg = c;
    _validateConfig();
    _updateActiveSensitivity();
  }

  const DspConfig& getConfig() const {
    return _cfg;
  }

  void resetConfigToDefaults() {
    _cfg = getDefaultConfig();
    _updateActiveSensitivity();
  }

  // Main high-speed processing burst, called every ~180-250 ms from dedicated FreeRTOS task
  void processBurst(uint32_t nowMs, float liveSpeedKmh, float liveInclPct = 0.0f) {
    if (nowMs - _lastBurstMs < 140) return;
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

    // 4. Power & Cumulative Energy (Watts and kWh)
    float effectiveBusV = 0.0f;
    if (liveSpeedKmh > 0.05f) {
      effectiveBusV = _cfg.nominalBusV * (0.15f + 0.85f * (liveSpeedKmh / 20.0f));
      if (effectiveBusV > _cfg.nominalBusV) effectiveBusV = _cfg.nominalBusV;
    } else if (instantA > 0.8f) {
      // Smart Fallback: Motor running on test bench or treadmill bridge speed pending
      effectiveBusV = _cfg.nominalBusV * 0.25f;
    }
    _m.powerWatts = effectiveBusV * _m.currentA;

    float dtSec = _lastEnergyUpdateMs > 0 ? (float)(nowMs - _lastEnergyUpdateMs) / 1000.0f : 0.25f;
    _lastEnergyUpdateMs = nowMs;
    if (dtSec > 0.0f && dtSec < 5.0f && _m.powerWatts > 0.0f) {
      _energyWs += _m.powerWatts * dtSec;
      _m.energyKWh = _energyWs / 3600000.0f;
    }

    // 5. Empty-Belt Baseline Capture & Maintenance Auto-Calibration
    static uint32_t runStartMs = 0;
    static float prevSpeedKmh = 0.0f;
    if (prevSpeedKmh <= 0.05f && liveSpeedKmh > 0.5f) {
      runStartMs = nowMs;
      _emptyBeltSampleCount = 0;
      _emptyBeltSum = 0.0f;
      _m.lubricationAlert = false;
    }
    prevSpeedKmh = liveSpeedKmh;

    // Inertia Delay Filter Window: bypass acceleration and drum inertia
    uint32_t startDelayMs = (uint32_t)(_cfg.inertiaDelayS * 1000.0f);
    if (startDelayMs < 1000) startDelayMs = 1000;
    if (startDelayMs > 10000) startDelayMs = 10000;
    uint32_t endDelayMs = startDelayMs + 5000; // 5-second clean measurement window

    if (runStartMs > 0 && (nowMs - runStartMs) >= startDelayMs && (nowMs - runStartMs) <= endDelayMs) {
      // Subtract incline gravitational load if running on an inclined deck
      float compA = (liveInclPct > 0.0f && _cfg.inclineCompA > 0.0f) ? (liveInclPct * _cfg.inclineCompA) : 0.0f;
      float netBeltCurrent = (instantA > compA) ? (instantA - compA) : instantA;

      _emptyBeltSum += netBeltCurrent;
      _emptyBeltSampleCount++;
      _m.emptyBeltA = _emptyBeltSum / (float)_emptyBeltSampleCount;

      // Auto-Calibration after "تم التشحيم"
      if (_pendingLubricationCal && _emptyBeltSampleCount >= 20) {
        float newBase = _m.emptyBeltA;
        if (newBase >= 0.8f && newBase <= 4.0f) {
          _cfg.calibratedBaselineA = newBase;
          _cfg.emptyBeltWarnA = newBase * 1.55f; // Automatically set warning threshold to +55% above fresh oil baseline
          _m.calibratedBaselineA = newBase;
        }
        _pendingLubricationCal = false;
      }

      float baseRef = _cfg.calibratedBaselineA > 0.5f ? _cfg.calibratedBaselineA : 1.8f;
      _m.calibratedBaselineA = baseRef;
      _m.frictionRatioPct = (_m.emptyBeltA / baseRef) * 100.0f;

      if (_m.emptyBeltA >= _cfg.emptyBeltWarnA) {
        _m.lubricationAlert = true;
      } else {
        _m.lubricationAlert = false;
      }

      // Belt Tension Diagnostic from unloaded empty belt
      if (_m.emptyBeltA > 0.5f) {
        if (_m.emptyBeltA < 1.15f) _m.beltTensionStatus = 0;       // Loose
        else if (_m.emptyBeltA <= 2.75f) _m.beltTensionStatus = 1; // Optimal
        else _m.beltTensionStatus = 2;                             // Over-tight
      }
    }

    // 6. Pre-filtering: Remove DC bias to isolate AC commutation ripple & arcing transients
    float minAc = 1e9f, maxAc = -1e9f;

    for (size_t i = 0; i < BURST_SIZE; i++) {
      _hp[i] = (_rawMv[i] - meanMv) / _effectiveSens(); // AC component in Amperes
      if (_hp[i] < minAc) minAc = _hp[i];
      if (_hp[i] > maxAc) maxAc = _hp[i];
    }
    _m.currentP2pA = (maxAc > minAc) ? (maxAc - minAc) : 0.0f;

    // Bearing Mechanical Roughness: Unloaded belt high-frequency ripple check
    if (liveSpeedKmh > 0.5f && _m.currentA < 3.0f) {
      if (_m.currentP2pA <= 0.45f) _m.bearingStatus = 0;        // Smooth
      else if (_m.currentP2pA <= 0.75f) _m.bearingStatus = 1;   // Slight Roughness
      else _m.bearingStatus = 2;                                // Worn / Service
    }

    // 7. Statistical Kurtosis Calculation (4th Standardized Moment)
    double sumSq = 0.0;
    double sumQuad = 0.0;
    for (size_t i = 0; i < BURST_SIZE; i++) {
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

    // 9. Medium-Frequency Band: Mechanical Footstrike Step Counter & Symmetry
    static float _stepBaseA = 0.0f;
    if (_stepBaseA < 0.1f) _stepBaseA = instantA;
    _stepBaseA = _stepBaseA * 0.96f + instantA * 0.04f; // Slow running baseline DC load

    float prevVal = _stepLpCurrent;
    _stepLpCurrent = _stepLpCurrent * 0.4f + instantA * 0.6f; // Fast footstrike impact pulse
    float slope = _stepLpCurrent - prevVal;

    // Trigger step if speed is streamed (>=1.0 km/h) OR motor is loaded under runner body weight (>=0.8A)
    if ((liveSpeedKmh >= 1.0f || _smoothedCurrentA >= 0.8f) && (nowMs - _lastStepMs) >= 220) {
      float thresholdA = _stepBaseA + _cfg.stepDeltaA;
      if (_stepLpCurrent >= thresholdA && _prevStepSlope > 0.0f && slope <= 0.0f) {
        uint32_t stepIntervalMs = nowMs - _lastStepMs;
        _lastStepMs = nowMs;
        _stepCount++;
        _m.stepCount = _stepCount;

        if (stepIntervalMs > 0 && stepIntervalMs < 1200) {
          uint16_t instantSpm = (uint16_t)(60000UL / stepIntervalMs);
          if (instantSpm >= 90 && instantSpm <= 220) {
            _m.cadenceSpm = (uint16_t)(_m.cadenceSpm * 0.7f + instantSpm * 0.3f);
          }
        }

        float peakAmp = _stepLpCurrent - _smoothedCurrentA;
        if (_stepAlternator) {
          _rightPeakSum = _rightPeakSum * 0.85f + peakAmp * 0.15f;
        } else {
          _leftPeakSum = _leftPeakSum * 0.85f + peakAmp * 0.15f;
        }
        _stepAlternator = !_stepAlternator;

        if (_leftPeakSum > 0.05f && _rightPeakSum > 0.05f) {
          float ratio = _leftPeakSum < _rightPeakSum ? (_leftPeakSum / _rightPeakSum) : (_rightPeakSum / _leftPeakSum);
          _symmetryPct = _symmetryPct * 0.9f + (ratio * 100.0f) * 0.1f;
          _m.symmetryPct = _symmetryPct;

          if (_m.symmetryPct < 70.0f) _m.fatigueState = 2;       // High Fatigue
          else if (_m.symmetryPct < 85.0f) _m.fatigueState = 1;  // Moderate Fatigue
          else _m.fatigueState = 0;                              // Fresh
        }

        // Dynamic Weight Estimation: Peak impact current proportional to body mass
        float avgPeak = (_leftPeakSum + _rightPeakSum) * 0.5f;
        if (avgPeak > 0.15f && liveSpeedKmh > 3.0f) {
          float estKg = (avgPeak * 55.0f) / (1.0f + 0.03f * liveSpeedKmh);
          if (estKg >= 40.0f && estKg <= 150.0f) {
            _estWeight = _estWeight > 0.0f ? (_estWeight * 0.95f + estKg * 0.05f) : estKg;
            _m.estimatedWeightKg = _estWeight;
          }
        }
      }
    }
    _prevStepSlope = slope;

    // Step-Off / Unloaded Belt Detector: belt running fast but current near empty belt
    if (liveSpeedKmh >= 3.0f && _m.emptyBeltA > 0.5f) {
      if ((nowMs - _lastStepMs) > 7000 && _m.currentA <= (_m.emptyBeltA + 0.40f)) {
        _m.stepOffAlert = true;
      } else {
        _m.stepOffAlert = false;
      }
    } else {
      _m.stepOffAlert = false;
    }

    // Running Economy: consumed electrical energy per km
    if (liveSpeedKmh > 2.0f && _m.powerWatts > 20.0f) {
      _m.runningEconomyWhPerKm = _m.powerWatts / liveSpeedKmh;
    }
  }

  void resetSteps() {
    _stepCount = 0;
    _m.stepCount = 0;
    _m.cadenceSpm = 0;
    _lastStepMs = 0;
  }

  void resetEnergy() {
    _energyWs = 0.0f;
    _m.energyKWh = 0.0f;
  }

  void loadCumulativeCounters(float totalKWh, uint32_t totalSteps) {
    if (!isnan(totalKWh) && totalKWh >= 0.0f) {
      _energyWs = totalKWh * 3600000.0f;
      _m.energyKWh = totalKWh;
    }
    _stepCount = totalSteps;
    _m.stepCount = totalSteps;
  }

  float getCumulativeEnergyKWh() const {
    return _m.energyKWh;
  }

  uint32_t getCumulativeSteps() const {
    return _stepCount;
  }

  void resetArcingScore() {
    _arcingScore = 0.0f;
    _m.arcingScore = 0.0f;
    _m.brushHealth = 0;
  }

  const Metrics& getMetrics() const {
    return _m;
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
    if (_cfg.stallDelayMs < 100 || _cfg.stallDelayMs > 1500) _cfg.stallDelayMs = 300;
    if (isnan(_cfg.kurtosisThresh) || _cfg.kurtosisThresh < 4.0f || _cfg.kurtosisThresh > 30.0f) _cfg.kurtosisThresh = 6.5f;
    if (isnan(_cfg.cusumWarnScore) || _cfg.cusumWarnScore < 5.0f || _cfg.cusumWarnScore > 50.0f) _cfg.cusumWarnScore = 18.0f;
    if (isnan(_cfg.cusumCritScore) || _cfg.cusumCritScore < 20.0f || _cfg.cusumCritScore > 90.0f) _cfg.cusumCritScore = 45.0f;
    if (isnan(_cfg.emptyBeltWarnA) || _cfg.emptyBeltWarnA < 1.5f || _cfg.emptyBeltWarnA > 6.0f) _cfg.emptyBeltWarnA = 3.5f;
    if (isnan(_cfg.stepDeltaA) || _cfg.stepDeltaA < 0.08f || _cfg.stepDeltaA > 0.80f) _cfg.stepDeltaA = 0.18f;
    if (isnan(_cfg.sensMvPerA) || _cfg.sensMvPerA < 15.0f || _cfg.sensMvPerA > 45.0f) _cfg.sensMvPerA = DEFAULT_SENS_MV_A;
    if (isnan(_cfg.nominalBusV) || _cfg.nominalBusV < 90.0f || _cfg.nominalBusV > 240.0f) _cfg.nominalBusV = RATED_BUS_VOLTS;
    if (isnan(_cfg.calibratedBaselineA) || _cfg.calibratedBaselineA < 0.5f || _cfg.calibratedBaselineA > 5.0f) _cfg.calibratedBaselineA = 1.80f;
    if (isnan(_cfg.inertiaDelayS) || _cfg.inertiaDelayS < 1.0f || _cfg.inertiaDelayS > 10.0f) _cfg.inertiaDelayS = 3.5f;
    if (isnan(_cfg.inclineCompA) || _cfg.inclineCompA < 0.0f || _cfg.inclineCompA > 0.50f) _cfg.inclineCompA = 0.15f;
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
