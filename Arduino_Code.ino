#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ================= LCD SETUP =================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change 0x27 to 0x3F if needed

// ================= MPU6050 SETUP =================
const int MPU_ADDR = 0x68;  // I2C address

int16_t accX, accY, accZ;

// Calibration offsets
float pitch_offset = 0.0;
float height_offset_cm = 0.0;

// Filtered (smoothed) values
float pitch_filt = 0.0;
float height_filt = 0.0;

// ================= ULTRASONIC SETUP =================
const int trigPin = 9;
const int echoPin = 8;

// ================= TIMING =================
const unsigned long SAMPLE_INTERVAL_MS = 100;  // 10 samples per second
unsigned long lastSampleTime = 0;
unsigned long startTime = 0;

// ================= FI PARAMETERS =================
// These are chosen so that:
// - On flat surface, even when moving smoothly, FI ≈ 100
// - Small obstacle gives a noticeable drop, but not all the way to 0

// Tolerance around flat after calibration
const float PITCH_FLAT_TOL_DEG = 2.0;   // degrees
const float HEIGHT_FLAT_TOL_CM = 0.4;   // cm

// Weights: how strongly pitch/height affect unevenness
const float W_PITCH  = 1.0;
const float W_HEIGHT = 4.0;  // height is more important

// Scale factor that converts unevenness score into FI drop
const float FI_DROP_SCALE = 6.0;

// Minimum FI (never show 0 unless it's a huge obstacle)
const float FI_MIN = 15.0;

// Smoothing factor for pitch/height (0..1), higher = more responsive, lower = smoother
const float FILTER_ALPHA = 0.3;

// ================ FUNCTIONS =================

void mpuWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();
  accZ = (Wire.read() << 8) | Wire.read();
}

float computePitchDeg() {
  float ax = accX;
  float ay = accY;
  float az = accZ;

  float pitch_rad = atan2(ax, sqrt(ay * ay + az * az));
  float pitch_deg = pitch_rad * 180.0 / PI;
  return pitch_deg;
}

float readUltrasonic_cm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL); // 30 ms timeout
  if (duration == 0) {
    return -1.0;
  }

  float distance_cm = duration / 58.0;
  return distance_cm;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050
  mpuWriteByte(0x6B, 0x00); // PWR_MGMT_1 = 0

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Flatness Rover");
  lcd.setCursor(0, 1);
  lcd.print("Calibrating...");

  // Calibration on flat surface (no motion)
  const int CAL_SAMPLES = 50; // ~5 seconds
  float pitchSum = 0.0;
  float heightSum = 0.0;
  int validCount = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    readMPU();
    float pitch = computePitchDeg();
    float h = readUltrasonic_cm();

    if (h > 0 && h < 300) {
      pitchSum += pitch;
      heightSum += h;
      validCount++;
    }
    delay(SAMPLE_INTERVAL_MS);
  }

  if (validCount > 0) {
    pitch_offset = pitchSum / validCount;
    height_offset_cm = heightSum / validCount;
  } else {
    pitch_offset = 0.0;
    height_offset_cm = 0.0;
  }

  // Initialize filters at zero deviation
  pitch_filt = 0.0;
  height_filt = 0.0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calib done");
  lcd.setCursor(0, 1);
  lcd.print("Ready to push");

  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FI: ");
  lcd.setCursor(0, 1);
  lcd.print("Ht:");

  lastSampleTime = millis();
  startTime = millis();

  Serial.println("time_ms,pitch_deg,height_cm,FI_percent");
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    unsigned long time_ms = now - startTime;

    // --- Read sensors ---
    readMPU();
    float pitch_raw = computePitchDeg();
    float height_raw_cm = readUltrasonic_cm();

    if (height_raw_cm <= 0 || height_raw_cm > 300) {
      // invalid ultrasonic reading, skip this cycle
      return;
    }

    // Deviation from calibrated flat baseline
    float pitch_dev = pitch_raw - pitch_offset;
    float height_dev = height_raw_cm - height_offset_cm;

    // Low-pass filter to remove vibration and spikes
    pitch_filt  = pitch_filt  + FILTER_ALPHA * (pitch_dev  - pitch_filt);
    height_filt = height_filt + FILTER_ALPHA * (height_dev - height_filt);

    float absP = fabs(pitch_filt);
    float absH = fabs(height_filt);

    float FI_percent = 100.0;

    // 1) If within flat tolerances -> always show 100%
    if (absP <= PITCH_FLAT_TOL_DEG && absH <= HEIGHT_FLAT_TOL_CM) {
      FI_percent = 100.0;
    } else {
      // 2) Compute "uneven score" only from the amount above the tolerances
      float excessP = absP - PITCH_FLAT_TOL_DEG;
      if (excessP < 0) excessP = 0;

      float excessH = absH - HEIGHT_FLAT_TOL_CM;
      if (excessH < 0) excessH = 0;

      float score = W_PITCH * excessP + W_HEIGHT * excessH;

      float drop = score * FI_DROP_SCALE;
      if (drop > 100.0) drop = 100.0;

      FI_percent = 100.0 - drop;
      if (FI_percent < FI_MIN) FI_percent = FI_MIN;  // don't go all the way to 0
    }

    // --- Send CSV line over Serial ---
    Serial.print(time_ms);
    Serial.print(",");
    Serial.print(pitch_filt, 3);   // log filtered pitch deviation
    Serial.print(",");
    Serial.print(height_filt, 3);  // log filtered height deviation
    Serial.print(",");
    Serial.println(FI_percent, 2);

    // --- Update LCD ---
    lcd.setCursor(0, 0);
    lcd.print("FI:");
    lcd.setCursor(3, 0);
    lcd.print("      ");
    lcd.setCursor(3, 0);
    lcd.print(FI_percent, 0);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("Ht:");
    lcd.setCursor(3, 1);
    lcd.print("       ");
    lcd.setCursor(3, 1);
    lcd.print(height_filt, 1);
    lcd.print("cm");
  }
}
