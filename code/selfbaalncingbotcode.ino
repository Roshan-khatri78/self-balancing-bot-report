#include <Wire.h>
#include <PID_v1.h>
#include <MPU6050_tockn.h>
#include <PinChangeInterrupt.h>

//----------------- Motor & Encoder Pin Definitions -----------------
#define ENA 6
#define IN1 8
#define IN2 10
#define ENB 9
#define IN3 11
#define IN4 12

#define ENCODER_L_A 2
#define ENCODER_L_B 3
#define ENCODER_R_A 4  
#define ENCODER_R_B 5  

//----------------- Global Variables -----------------
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
volatile long lastEncoderLeftCount = 0;
volatile long lastEncoderRightCount = 0;

// Setpoint remains zero (upright)
double setpoint = 0;

// PID input (scaled tilt) and output (motor command)
double input = 0;
double output = 0;

//----- NEW: Angle Sensitivity -----
// Amplify small tilts (e.g., with angleSensitivity = 5, a 1° tilt produces a 5 unit error)
const double angleSensitivity = 5.0;

//----- PID parameters -----
double Kp = 25, Ki = 0, Kd = 5;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

unsigned long lastPrintTime = 0;
bool autoTuning = false;  // Toggle auto tuning mode

MPU6050 mpu(Wire);

// Encoder correction factor – adjust if necessary
double kEnc = 0.8;  // Reduced from 1.5

//----- Complementary Filter Variables -----
// Initializes with the first angle reading
double filteredAngle = 0.0;

// Minimal PWM threshold to overcome motor deadband
const int minPWM = 30;  // Increased from 20

//----------------- Encoder ISRs -----------------
void encoderLeftISR() {
  int b = digitalRead(ENCODER_L_B);
  (b == HIGH) ? encoderLeftCount++ : encoderLeftCount--;
}

void encoderRightISR() {
  int b = digitalRead(ENCODER_R_B);
  (b == HIGH) ? encoderRightCount++ : encoderRightCount--;
}

//----------------- Motor Control with Encoder Correction -----------------
void setMotorSpeedWithEncoder(double baseSpeed) {
  noInterrupts();
  long dLeft = encoderLeftCount - lastEncoderLeftCount;
  long dRight = encoderRightCount - lastEncoderRightCount;
  lastEncoderLeftCount = encoderLeftCount;
  lastEncoderRightCount = encoderRightCount;
  interrupts();
  
  // Encoder correction to help balance motor speeds
  double correction = kEnc * (dLeft - dRight);
  double leftSpeed = baseSpeed - correction;
  double rightSpeed = baseSpeed + correction;
  
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  int pwmLeft = abs(leftSpeed);
  int pwmRight = abs(rightSpeed);
  
  if (pwmLeft > 0 && pwmLeft < minPWM) pwmLeft = minPWM;
  if (pwmRight > 0 && pwmRight < minPWM) pwmRight = minPWM;
  
  //--- Motor Control with reversed polarity ---
  // Left motor (reversed polarity)
  if (leftSpeed > 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwmLeft);
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmLeft);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  
  // Right motor (reversed polarity)
  if (rightSpeed > 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwmRight);
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmRight);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
  
  // Serial output for debugging every 200ms
  if (millis() - lastPrintTime >= 200) {
    Serial.print("Filtered Angle: "); Serial.print(filteredAngle, 2);
    Serial.print(" | Scaled Input: "); Serial.print(input, 2);
    Serial.print(" | L Speed: "); Serial.print(leftSpeed, 1);
    Serial.print(" | R Speed: "); Serial.println(rightSpeed, 1);
    lastPrintTime = millis();
  }
}

//----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  pinMode(ENCODER_L_A, INPUT_PULLUP); pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP); pinMode(ENCODER_R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderLeftISR, CHANGE);
  attachPCINT(ENCODER_R_A, encoderRightISR, CHANGE);
  
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("MPU6050 ready");
  delay(3000);
  
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(10);
  
  // Initialize the filter with the first reading (invert so forward tilt is positive)
  filteredAngle = -mpu.getAngleY();
}

//----------------- Main Loop -----------------
void loop() {
  mpu.update();
  // Read raw angle (invert so forward tilt is positive)
  double rawAngle = -mpu.getAngleY();
  
  // Complementary filtering: 0.98 weight for gyro integration and 0.02 for accelerometer
  filteredAngle = 0.98 * (filteredAngle + mpu.getGyroY() * 0.01) + 0.02 * rawAngle;
  
  // Scale the filtered angle so that a small tilt produces a larger error for PID
  input = filteredAngle * angleSensitivity;
  
  // Toggle auto tuning mode via serial command if needed
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'a') {
      autoTuning = !autoTuning;
      Serial.print("AutoTuning: ");
      Serial.println(autoTuning ? "ON" : "OFF");
    }
  }
  
  if (autoTuning) {
    setMotorSpeedWithEncoder(0);
    if (millis() - lastPrintTime >= 200) {
      Serial.print("AutoTuning Mode - Scaled Input: "); Serial.println(input, 2);
      lastPrintTime = millis();
    }
  } else {
    pid.Compute();
    setMotorSpeedWithEncoder(output);
  }
  
  delay(10);
}
