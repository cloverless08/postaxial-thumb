#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#define FIRMWARE_VERSION       "1.1.0"

#define FLEX_PIN               39
#define SERVO_PIN              26
#define SDA_PIN                21
#define SCL_PIN                22

#define SCREEN_WIDTH           128
#define SCREEN_HEIGHT          64
#define OLED_RESET             -1
#define SCREEN_ADDRESS         0x3C

#define SERVO_MIN_US           500
#define SERVO_MAX_US           2400
#define SERVO_FREQ_HZ          50
#define SERVO_IDLE_MS          2000    // detach after this many ms of no movement

#define FLEX_STRAIGHT_DEFAULT  1800    // set via calibration, or adjust these defaults
#define FLEX_BENT_DEFAULT      3200

#define ALPHA                  0.25f   // EMA smoothing: 0.05 (glassy) -> 0.3 (snappy)
#define DEAD_BAND_DEG          2       // ignore changes smaller than this
#define MAX_DEG_PER_TICK       30      // max servo travel per loop tick

#define INVERT_DIRECTION       false   // flip if servo is mounted backwards

#define SAMPLE_COUNT           16
#define SAMPLE_INTERVAL_MS     2

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Servo myServo;

int   flexStraight   = FLEX_STRAIGHT_DEFAULT;
int   flexBent       = FLEX_BENT_DEFAULT;
float smoothedAngle  = 90.0f;
int   lastServoAngle = 90;
bool  servoAttached  = true;
unsigned long lastMoveTime = 0;

// Non-blocking sampler state
int  sampleAccum   = 0;
int  samplesTaken  = 0;
unsigned long lastSampleTime = 0;
int  latestRaw     = 0;

// Returns true once every SAMPLE_COUNT readings, with the average in latestRaw
bool updateSampler() {
  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) return false;
  lastSampleTime = millis();
  sampleAccum += analogRead(FLEX_PIN);
  if (++samplesTaken < SAMPLE_COUNT) return false;
  latestRaw   = sampleAccum / SAMPLE_COUNT;
  sampleAccum = 0;
  samplesTaken = 0;
  return true;
}

void attachServo() {
  if (servoAttached) return;
  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoAttached = true;
}

void detachIfIdle() {
  if (servoAttached && millis() - lastMoveTime > SERVO_IDLE_MS) {
    myServo.detach();
    servoAttached = false;
  }
}

void runCalibration() {
  Serial.println("\n=== CALIBRATION MODE ===");
  Serial.println("Hold finger FLAT, send 'F'");
  Serial.println("Hold finger BENT, send 'B'");
  Serial.println("Send 'X' to exit\n");

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.println("CALIBRATION MODE");
  display.setCursor(0, 16); display.println("F = flat  B = bent");
  display.setCursor(0, 32); display.println("X = exit");
  display.display();

  while (true) {
    int raw = analogRead(FLEX_PIN);
    Serial.print("Raw: "); Serial.println(raw);

    if (Serial.available()) {
      char cmd = toupper(Serial.read());
      if (cmd == 'F') {
        flexStraight = raw;
        Serial.print("FLAT set: "); Serial.println(flexStraight);
      } else if (cmd == 'B') {
        flexBent = raw;
        Serial.print("BENT set: "); Serial.println(flexBent);
      } else if (cmd == 'X') {
        Serial.print("Done. FLAT="); Serial.print(flexStraight);
        Serial.print(" BENT=");      Serial.println(flexBent);
        break;
      }
    }
    delay(100);
  }
}

void debugPrint(int raw, int target, int smoothed) {
  Serial.print("Raw:"); Serial.print(raw);
  Serial.print("  Target:"); Serial.print(target);
  Serial.print("deg  Smoothed:"); Serial.print(smoothed);
  Serial.println("deg");
}

void updateDisplay(int raw, int angle) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.println("TMNT Flex Sensor");
  display.setCursor(0, 18); display.print("Raw:   "); display.println(raw);
  display.setCursor(0, 30); display.print("Angle: "); display.print(angle); display.println((char)247);
  int barWidth = map(angle, 0, 180, 0, SCREEN_WIDTH);
  display.fillRect(0, 52, barWidth,     10, SSD1306_WHITE);
  display.drawRect(0, 52, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Firmware v" FIRMWARE_VERSION);
  Serial.println("Send 'C' to calibrate");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(SERVO_FREQ_HZ);
  myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  myServo.write(90);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED not found");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println("Flex -> Servo Ready");
  display.setCursor(0, 16); display.print("v"); display.println(FIRMWARE_VERSION);
  display.setCursor(0, 32); display.println("Talk Nerdy To Me!");
  display.display();
  delay(100);
}

void loop() {
  if (Serial.available() && toupper(Serial.peek()) == 'C') {
    Serial.read();
    runCalibration();
  }

  if (!updateSampler()) return;

  int targetAngle = map(latestRaw, flexStraight, flexBent, 0, 180);
  targetAngle = constrain(targetAngle, 0, 180);
  if (INVERT_DIRECTION) targetAngle = 180 - targetAngle;

  smoothedAngle = ALPHA * targetAngle + (1.0f - ALPHA) * smoothedAngle;
  int newAngle = (int)round(smoothedAngle);

  if (abs(newAngle - lastServoAngle) < DEAD_BAND_DEG) {
    detachIfIdle();
    updateDisplay(latestRaw, lastServoAngle);
    debugPrint(latestRaw, targetAngle, lastServoAngle);
    return;
  }

  // Clamp movement rate to protect the servo mechanically
  int delta = constrain(newAngle - lastServoAngle, -MAX_DEG_PER_TICK, MAX_DEG_PER_TICK);
  newAngle = lastServoAngle + delta;

  attachServo();
  myServo.write(newAngle);
  lastServoAngle = newAngle;
  lastMoveTime   = millis();

  updateDisplay(latestRaw, newAngle);
  debugPrint(latestRaw, targetAngle, newAngle);
}
