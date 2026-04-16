// Arduino Uno Stepper + Encoder Simulator for Giga OptiDrive v6.6
// v1.10 — Motor steps and Encoder printed with inverted polarity (negative values)

#include <Arduino.h>

#define STEP_IN     2
#define DIR_IN      3
#define ENABLE_IN   4

#define A_OUT       5
#define B_OUT       6
#define Z_OUT       7

#define MOTOR_SOUND_PIN 11



volatile long currentSubstep = 0;

void stepISR() {
  if (digitalRead(ENABLE_IN) == HIGH) {
    // Inverted polarity: +1 when DIR LOW
    currentSubstep += (digitalRead(DIR_IN) == LOW ? 1L : -1L);
  }
}

int getPhase(long sub) {
  // phase now computed from sub/4L as you changed
  long quarters = ((sub / 4L) * 25L) / 128L;
  return (int)((quarters % 4 + 4) % 4);
}

long getEncoderCounts(long sub) {
  return (sub * 25L) / 512L;               // 10,000 counts per 204800 steps
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("=== Uno Stepper + Encoder Simulator v1.10 ==="));
  Serial.println(F("204800 steps/rev → exactly 10000 encoder counts/rev"));
  Serial.println(F("Quadrature (A leads B forward) + Z-index pulse"));
  Serial.println(F("DIR polarity inverted (forward when DIR=LOW)"));
  Serial.println(F("Motor steps & Encoder printed with inverted polarity (negative)"));
  Serial.println(F("Wiring: Giga 9→Uno2, 8→3, 7→4 | Uno5→Giga2, 6→3, 7→5"));

  pinMode(STEP_IN,   INPUT);
  pinMode(DIR_IN,    INPUT);
  pinMode(ENABLE_IN, INPUT);
  pinMode(A_OUT, OUTPUT);
  pinMode(B_OUT, OUTPUT);
  pinMode(Z_OUT, OUTPUT);

  pinMode (MOTOR_SOUND_PIN, OUTPUT);

  digitalWrite(A_OUT, LOW);
  digitalWrite(B_OUT, LOW);
  digitalWrite(Z_OUT, HIGH);

  attachInterrupt(digitalPinToInterrupt(STEP_IN), stepISR, RISING);

  Serial.println(F("Simulator READY. Test Giga v6.6 commands now."));
}

void loop() {
  static long lastSub = -1;
  long sub = currentSubstep;

  if (sub != lastSub) {

        // phase computed from sub/4L
    int phase = getPhase(sub );

    bool A = LOW, B = LOW;
    switch (phase) {
      case 0: A = LOW;  B = LOW;  break;
      case 1: A = HIGH; B = LOW;  break;
      case 2: A = HIGH; B = HIGH; break;
      case 3: A = LOW;  B = HIGH; break;
    }
    digitalWrite(A_OUT, A);
    delayMicroseconds(10);         // helps ensure clean 4× decoding on Giga
    digitalWrite(B_OUT, B);
    delayMicroseconds(10);
    lastSub = sub;
  }

  long enc = getEncoderCounts(sub);
  long mod = ((enc % 10000L) + 10000L) % 10000L;
  digitalWrite(Z_OUT, ((mod < 2600)&&(mod > 2500)) ? HIGH : LOW);

  // === Motor sound: pitch rises with step rate ===
  static unsigned long lastSoundTime = 0;
  unsigned long soundnow = micros();

  if (soundnow - lastSoundTime >= 20000) {  // update sound every 10 ms
    // Only play if moving (speed > 0) and enabled
    if (digitalRead(ENABLE_IN) == HIGH && abs(currentSubstep - lastSub) > 0) {
      // Frequency proportional to absolute step rate (higher speed = higher pitch)
      // Base: 400 Hz at low speed, up to ~2000 Hz at high speed
      long stepRate = abs(currentSubstep - lastSub) * 100;  // rough scaling
      int freq = constrain(300 + stepRate, 300, 2000);
      tone(MOTOR_SOUND_PIN, freq);
    } else {
      noTone(MOTOR_SOUND_PIN);  // stop sound when idle
    }
    lastSoundTime = soundnow;
  }
  // === Conditional Status Report with pause time on new line ===
  static unsigned long lastReport = 0;
  static long lastReportedSub = -1;
  static long lastReportedEnc = -1;

  unsigned long now = millis();

  if ((sub != lastReportedSub || enc != lastReportedEnc) && (now - lastReport >= 2000)) {
    long motorPos  = ((sub % 204800L) + 204800L) % 204800L / 20480L;
    float motorDegrees  = ((sub % 204800L) + 204800L) % 204800L / 568.889L;
    long encoderPos = ((enc % 10000L) + 10000L) % 10000L / 1000L;

    unsigned long deltaTime = now - lastReport;

    // Print time gap on its own line if there was a pause
    if (deltaTime > 2500) {
      Serial.print(F("Δt: "));
      Serial.print(deltaTime);
      Serial.println(F(" ms"));
    }

    // Main report line — Motor steps and Encoder now printed as negative values
    Serial.print(F("Motor steps: ,"));
    Serial.print(sub);                    // ← polarity inverted in report
    Serial.print(F(", | Encoder: ,"));
    Serial.print(enc);                    // ← polarity inverted in report
    Serial.print(F(", | MotorPos: ,"));
    Serial.print(motorPos);
    Serial.print(F(", | EncPos: ,"));
    Serial.print(encoderPos);
    Serial.print(F(", | Z: ,"));
    Serial.print(digitalRead(Z_OUT) == HIGH ? "HIGH" : "low");
    Serial.print(F(", | MotorDegrees: ,"));
    Serial.print(motorDegrees,2 );
    Serial.println();

    lastReportedSub = sub;
    lastReportedEnc = enc;
    lastReport = now;
  }
}