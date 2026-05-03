// Arduino Uno Stepper + Encoder Simulator for Giga OptiDrive v6.6
// v1.12 — Added step-timing jitter analysis monitor

#include <Arduino.h>

// ── Pin assignments ──────────────────────────────────────────────────────────
#define STEP_IN          2
#define DIR_IN           3
#define ENABLE_IN        4
#define A_OUT            5
#define B_OUT            6
#define Z_OUT            7
#define MOTOR_SOUND_PIN  11

// ── Motor / drivetrain constants ─────────────────────────────────────────────
#define MOTOR_STEPS_PER_REV  800UL    // motor shaft: 200 full-step × 4 microstep
#define GEAR_RATIO           256UL    // gearbox reduction to output shaft
#define OUTPUT_STEPS_PER_REV (MOTOR_STEPS_PER_REV * GEAR_RATIO)  // 204,800
#define MOTOR_PULLOUT_RPM    500UL    // approx max motor-shaft RPM before step loss

// ── Jitter detection thresholds (step-to-step comparison) ───────────────────
// If a step arrives >MISS_PCT% shorter than the previous interval, the motor
// cannot have completed its previous step — the command would be skipped.
#define JITTER_WARN_PCT  15    // interval shrinks >15% vs previous → warning
#define JITTER_MISS_PCT  30    // interval shrinks >30% vs previous → missed-step risk
#define IDLE_TIMEOUT_MS  50    // ms silence after last step → move ended

// ── Ring buffer for last N step intervals ────────────────────────────────────
// Stored in µs (capped at 65535).  64 entries = 128 bytes RAM.
#define JBUF_SIZE  64          // must be a power of 2
volatile uint16_t jBuf[JBUF_SIZE];
volatile uint8_t  jHead = 0;   // next-write index
volatile uint8_t  jFill = 0;   // valid entries (0‥JBUF_SIZE)

// ── Per-move statistics (accumulated by the ISR across the whole move) ───────
volatile bool     isr_inMove    = false;  // true after first step of a move
volatile uint32_t isr_stepCount = 0;      // interval count (steps − 1)
volatile uint32_t isr_sumUs     = 0;      // sum of intervals (µs)
volatile uint16_t isr_minUs     = 65535U;
volatile uint16_t isr_maxUs     = 0;
volatile uint16_t isr_prevUs    = 0;      // previous interval for step-to-step diff
volatile uint16_t isr_warnCount = 0;      // events >15% shorter than previous
volatile uint16_t isr_missCount = 0;      // events >30% shorter than previous
volatile uint32_t isr_lastUs    = 0;      // micros() of last step

// ── Encoder / quadrature ─────────────────────────────────────────────────────
volatile long currentSubstep = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  ISR — fires on every STEP_IN rising edge
// ─────────────────────────────────────────────────────────────────────────────
void stepISR() {
  uint32_t now     = micros();
  bool     enabled = (digitalRead(ENABLE_IN) == HIGH);

  if (enabled) {
    currentSubstep += (digitalRead(DIR_IN) == LOW ? 1L : -1L);

    if (isr_inMove) {
      // ── Compute interval ─────────────────────────────────────────────────
      uint32_t dt32 = now - isr_lastUs;
      uint16_t dt   = (dt32 > 65535UL) ? 65535U : (uint16_t)dt32;

      // Push to ring buffer (overwrites oldest when full)
      jBuf[jHead] = dt;
      jHead = (jHead + 1) & (JBUF_SIZE - 1);
      if (jFill < JBUF_SIZE) jFill++;

      // Full-move accumulators
      isr_stepCount++;
      if (isr_sumUs < 0xFFFF0000UL) isr_sumUs += dt;   // overflow guard
      if (dt < isr_minUs) isr_minUs = dt;
      if (dt > isr_maxUs) isr_maxUs = dt;

      // Step-to-step jitter check (integer multiply avoids division in ISR)
      // cur * 10 < prev * 7  →  cur < 0.70 × prev  →  >30% shorter → missed
      // cur * 20 < prev * 17 →  cur < 0.85 × prev  →  >15% shorter → warn
      if (isr_prevUs > 0) {
        uint32_t cur = dt;
        uint32_t prv = isr_prevUs;
        if      (cur * 10UL < prv * 7UL)       isr_missCount++;
        else if (cur * 20UL < prv * 17UL)       isr_warnCount++;
      }
      isr_prevUs = dt;
    }

    isr_inMove = true;
    isr_lastUs = now;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Quadrature helpers
// ─────────────────────────────────────────────────────────────────────────────
int getPhase(long sub) {
  long quarters = ((sub / 4L) * 25L) / 128L;
  return (int)((quarters % 4 + 4) % 4);
}

long getEncoderCounts(long sub) {
  return (sub * 25L) / 512L;   // 10,000 encoder counts per 204,800 steps
}

// ─────────────────────────────────────────────────────────────────────────────
//  Print jitter analysis report (called only during idle — safe to Serial.print)
// ─────────────────────────────────────────────────────────────────────────────
void printJitterReport(uint32_t moveNum,
                       uint32_t stepCount, uint32_t sumUs,
                       uint16_t minUs,     uint16_t maxUs,
                       uint16_t warnCnt,   uint16_t missCnt,
                       uint8_t  bufFill,   uint16_t* buf)
{
  if (stepCount < 2) {
    Serial.println(F("[Jitter] Too few steps to analyse."));
    return;
  }

  uint32_t meanUs = sumUs / stepCount;

  // ── Speed ──────────────────────────────────────────────────────────────────
  uint32_t stepsPerSec    = (meanUs > 0) ? (1000000UL / meanUs) : 0;
  float    motorRPM       = (float)stepsPerSec * 60.0f / (float)MOTOR_STEPS_PER_REV;
  float    outputRPM      = motorRPM            / (float)GEAR_RATIO;

  // ── Peak deviations from mean ──────────────────────────────────────────────
  int32_t peakEarlyPct = (meanUs > 0 && minUs < meanUs)
                         ? (int32_t)(meanUs - minUs) * 100L / (int32_t)meanUs : 0;
  int32_t peakLatePct  = (meanUs > 0 && maxUs > meanUs)
                         ? (int32_t)(maxUs - meanUs) * 100L / (int32_t)meanUs : 0;

  // ── Mean-absolute-deviation from ring buffer (last bufFill intervals) ──────
  uint32_t madSum = 0;
  for (uint8_t i = 0; i < bufFill; i++) {
    int32_t d = (int32_t)buf[i] - (int32_t)meanUs;
    madSum += (uint32_t)(d < 0 ? -d : d);
  }
  float mad = (bufFill > 0) ? (float)madSum / bufFill : 0.0f;

  // ── Pullout / over-speed check ─────────────────────────────────────────────
  uint32_t pulloutStepsPerSec = MOTOR_PULLOUT_RPM * MOTOR_STEPS_PER_REV / 60UL;
  bool     overSpeed          = (stepsPerSec > pulloutStepsPerSec);

  // ── Status ─────────────────────────────────────────────────────────────────
  const __FlashStringHelper* status;
  if      (missCnt > 0 || overSpeed) status = F("CRITICAL");
  else if (warnCnt > 0)              status = F("WARNING");
  else                               status = F("OK");

  // ── Print ──────────────────────────────────────────────────────────────────
  Serial.println();
  Serial.print(F("=== Jitter Analysis — Move #")); Serial.print(moveNum); Serial.println(F(" ==="));

  Serial.print(F("  Intervals sampled : ")); Serial.println(stepCount);
  Serial.print(F("  Mean interval     : ")); Serial.print(meanUs);   Serial.println(F(" µs"));
  Serial.print(F("  Motor shaft       : ")); Serial.print(motorRPM, 1); Serial.println(F(" RPM"));
  Serial.print(F("  Output shaft      : ")); Serial.print(outputRPM, 3); Serial.println(F(" RPM"));
  Serial.println();
  Serial.print(F("  Min interval      : ")); Serial.print(minUs); Serial.println(F(" µs"));
  Serial.print(F("  Max interval      : ")); Serial.print(maxUs); Serial.println(F(" µs"));
  Serial.print(F("  Peak early jitter : -")); Serial.print(peakEarlyPct); Serial.println(F("%"));
  Serial.print(F("  Peak late jitter  : +")); Serial.print(peakLatePct);  Serial.println(F("%"));
  Serial.print(F("  Mean abs deviation: ")); Serial.print(mad, 1);
  Serial.print(F(" µs (last ")); Serial.print(bufFill); Serial.println(F(" steps)"));
  Serial.println();
  Serial.print(F("  Warn  (>"));  Serial.print(JITTER_WARN_PCT);
  Serial.print(F("% step-to-step shrink): ")); Serial.println(warnCnt);
  Serial.print(F("  Missed(>"));  Serial.print(JITTER_MISS_PCT);
  Serial.print(F("% step-to-step shrink): ")); Serial.println(missCnt);
  Serial.println();
  Serial.print(F("  Speed zone        : "));
  if (overSpeed) {
    Serial.print(F("OVER LIMIT  (motor pullout ~"));
    Serial.print(MOTOR_PULLOUT_RPM);
    Serial.println(F(" RPM)"));
  } else {
    Serial.println(F("safe"));
  }
  Serial.print(F("  >>> STATUS: ")); Serial.println(status);
  Serial.println(F("---"));
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("=== Uno Stepper + Encoder Simulator v1.12 ==="));
  Serial.println(F("204800 steps/rev (800 steps/motor-rev × 256:1 gearbox)"));
  Serial.println(F("Quadrature (A leads B forward) + Z-index pulse"));
  Serial.println(F("DIR polarity inverted (forward when DIR=LOW)"));
  Serial.println(F("Motor steps & Encoder printed with inverted polarity (negative)"));
  Serial.println(F("Wiring: Giga 9→Uno2, 8→3, 7→4 | Uno5→Giga2, 6→3, 7→5"));
  Serial.println(F("Jitter monitor: ENABLED — report printed after each idle period"));

  pinMode(STEP_IN,   INPUT);
  pinMode(DIR_IN,    INPUT);
  pinMode(ENABLE_IN, INPUT);
  pinMode(A_OUT,     OUTPUT);
  pinMode(B_OUT,     OUTPUT);
  pinMode(Z_OUT,     OUTPUT);
  pinMode(MOTOR_SOUND_PIN, OUTPUT);

  digitalWrite(A_OUT, LOW);
  digitalWrite(B_OUT, LOW);
  digitalWrite(Z_OUT, HIGH);

  attachInterrupt(digitalPinToInterrupt(STEP_IN), stepISR, RISING);

  Serial.println(F("Simulator READY."));
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  static long          lastSub        = -1;
  static long          lastReportSub  = -1;
  static long          lastReportEnc  = -1;
  static unsigned long lastReport     = 0;
  static unsigned long lastStepMs     = 0;
  static unsigned long lastSoundTime  = 0;
  static bool          wasMoving      = false;
  static uint32_t      moveNumber     = 0;

  long         sub   = currentSubstep;
  unsigned long nowMs = millis();

  // ── Track motion / idle ───────────────────────────────────────────────────
  if (sub != lastSub) {
    lastStepMs = nowMs;
    wasMoving  = true;
  }

  // ── Idle detected → snapshot stats and print jitter report ───────────────
  if (wasMoving && (nowMs - lastStepMs > IDLE_TIMEOUT_MS)) {
    wasMoving = false;
    moveNumber++;

    // Atomically snapshot all ISR state
    noInterrupts();
    uint32_t snapCount = isr_stepCount;
    uint32_t snapSum   = isr_sumUs;
    uint16_t snapMin   = isr_minUs;
    uint16_t snapMax   = isr_maxUs;
    uint16_t snapWarn  = isr_warnCount;
    uint16_t snapMiss  = isr_missCount;
    uint8_t  snapFill  = jFill;
    uint8_t  snapHead  = jHead;

    // Copy ring buffer in chronological order
    uint16_t localBuf[JBUF_SIZE];
    for (uint8_t i = 0; i < snapFill; i++) {
      uint8_t idx = (uint8_t)((snapHead - snapFill + i + JBUF_SIZE) & (JBUF_SIZE - 1));
      localBuf[i] = jBuf[idx];
    }

    // Reset for next move
    isr_stepCount = 0;   isr_sumUs  = 0;
    isr_minUs = 65535U;  isr_maxUs  = 0;
    isr_prevUs = 0;
    isr_warnCount = 0;   isr_missCount = 0;
    jFill = 0;           jHead = 0;
    isr_inMove = false;
    interrupts();

    printJitterReport(moveNumber, snapCount, snapSum, snapMin, snapMax,
                      snapWarn, snapMiss, snapFill, localBuf);
  }

  // ── Quadrature output ─────────────────────────────────────────────────────
  if (sub != lastSub) {
    int phase = getPhase(sub);
    bool A = LOW, B = LOW;
    switch (phase) {
      case 0: A = LOW;  B = LOW;  break;
      case 1: A = HIGH; B = LOW;  break;
      case 2: A = HIGH; B = HIGH; break;
      case 3: A = LOW;  B = HIGH; break;
    }
    digitalWrite(A_OUT, A);
    delayMicroseconds(10);
    digitalWrite(B_OUT, B);
    delayMicroseconds(10);
    lastSub = sub;
  }

  // ── Z-index pulse ─────────────────────────────────────────────────────────
  long enc = getEncoderCounts(sub);
  long mod = ((enc % 10000L) + 10000L) % 10000L;
  digitalWrite(Z_OUT, ((mod < 2600) && (mod > 2500)) ? HIGH : LOW);

  // ── Motor sound: pitch rises with step rate ───────────────────────────────
  unsigned long soundNow = micros();
  if (soundNow - lastSoundTime >= 20000) {
    if (digitalRead(ENABLE_IN) == HIGH && abs(currentSubstep - lastSub) > 0) {
      long stepRate = abs(currentSubstep - lastSub) * 100;
      int  freq     = constrain(300 + (int)stepRate, 300, 2000);
      tone(MOTOR_SOUND_PIN, freq);
    } else {
      noTone(MOTOR_SOUND_PIN);
    }
    lastSoundTime = soundNow;
  }

  // ── Periodic status report (2-second cadence) ────────────────────────────
  if ((sub != lastReportSub || enc != lastReportEnc) && (nowMs - lastReport >= 2000)) {
    long  motorPos      = ((sub % 204800L) + 204800L) % 204800L / 20480L;
    float motorDegrees  = ((sub % 204800L) + 204800L) % 204800L / 568.889f;
    long  encoderPos    = ((enc % 10000L)  + 10000L)  % 10000L  / 1000L;
    unsigned long deltaTime = nowMs - lastReport;

    if (deltaTime > 2500) {
      Serial.print(F("Δt: ")); Serial.print(deltaTime); Serial.println(F(" ms"));
    }

    Serial.print(F("Motor steps: ,"));         Serial.print(sub);
    Serial.print(F(", | Encoder: ,"));          Serial.print(enc);
    Serial.print(F(", | MotorPos: ,"));          Serial.print(motorPos);
    Serial.print(F(", | EncPos: ,"));            Serial.print(encoderPos);
    Serial.print(F(", | Z: ,"));
    Serial.print(digitalRead(Z_OUT) == HIGH ? "HIGH" : "low");
    Serial.print(F(", | MotorDegrees: ,"));      Serial.print(motorDegrees, 2);
    Serial.println();

    lastReportSub = sub;
    lastReportEnc = enc;
    lastReport    = nowMs;
  }
}
