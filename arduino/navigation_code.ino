#include <Wire.h>
#include <MPU9250.h>
#include <util/atomic.h>

// IMU
MPU9250 mpu;
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
float ax, ay, az, gx, gy, gz;

// Motor Right
#define ENCA 3 //yellow
#define IN1 9
#define IN2 8
#define PWM 10

// Motor Left
#define ENCA_LEFT 2  //yellow
#define IN3_LEFT 6
#define IN4_LEFT 7
#define PWM_LEFT 11

volatile int pos_i = 0, pos_i_LEFT = 0;
volatile float velocity_i = 0, velocity_i_LEFT = 0;
volatile long prevT_i = 0, prevT_i_LEFT = 0;

float v1Filt = 0, v1Prev = 0;
float v1Filt_LEFT = 0, v1Prev_LEFT = 0;
float eintegral = 0, eintegral_LEFT = 0;

float vt = 0, vt_LEFT = 0;
long prevT = 0, prevT_LEFT = 0;
int posPrev = 0, posPrev_LEFT = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 not found");
    while (1);
  }

  pinMode(ENCA, INPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(PWM, OUTPUT);
  pinMode(ENCA_LEFT, INPUT); pinMode(IN3_LEFT, OUTPUT); pinMode(IN4_LEFT, OUTPUT); pinMode(PWM_LEFT, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_LEFT), readEncoder_LEFT, RISING);
}

void loop() {
  // IMU update
  if (mpu.update()) {
    ax = mpu.getAccX(); ay = mpu.getAccY(); az = mpu.getAccZ();
    gx = mpu.getGyroX(); gy = mpu.getGyroY(); gz = mpu.getGyroZ();
    updateOrientation(gx, gy, gz, ax, ay, az);
  }

  // Velocity calculation
  int pos = 0, pos_LEFT = 0;
  float velocity2 = 0, velocity2_LEFT = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i; velocity2 = velocity_i;
    pos_LEFT = pos_i_LEFT; velocity2_LEFT = velocity_i_LEFT;
  }

  long currT = micros();
  float deltaT = (currT - prevT) / 1.0e6;
  float velocity1 = (pos - posPrev) / deltaT;
  posPrev = pos; prevT = currT;

  long currT_LEFT = micros();
  float deltaT_LEFT = (currT_LEFT - prevT_LEFT) / 1.0e6;
  float velocity1_LEFT = (pos_LEFT - posPrev_LEFT) / deltaT_LEFT;
  posPrev_LEFT = pos_LEFT; prevT_LEFT = currT_LEFT;


  // float v1 = velocity1 / 270.0 * 60.0;
  // float v1_LEFT = velocity1_LEFT / 270.0 * 60.0;


  float v1 = (velocity1 / 270.0 * 60.0) * (vt >= 0 ? 1 : -1);
float v1_LEFT = (velocity1_LEFT / 270.0 * 60.0) * (vt_LEFT >= 0 ? 1 : -1);


  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev; v1Prev = v1;
  v1Filt_LEFT = 0.854 * v1Filt_LEFT + 0.0728 * v1_LEFT + 0.0728 * v1Prev_LEFT; v1Prev_LEFT = v1_LEFT;

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      vt = input.substring(0, spaceIndex).toFloat();
      vt_LEFT = input.substring(spaceIndex + 1).toFloat();
    }
  }


  float kp = 1, ki = 2;


// Reset integral term if there's a large change in direction
if (abs(vt) > 0 && abs(v1Filt) < 0.5) {
  eintegral = 0;
  eintegral_LEFT = 0;
}



 eintegral += (vt - v1Filt) * deltaT;
eintegral = constrain(eintegral, -100, 100);  // Clamp after updating

eintegral_LEFT += (vt_LEFT - v1Filt_LEFT) * deltaT_LEFT;
eintegral_LEFT = constrain(eintegral_LEFT, -100, 100);  // Clamp after updating

float u = kp * (vt - v1Filt) + ki * eintegral;
float u_LEFT = kp * (vt_LEFT - v1Filt_LEFT) + ki * eintegral_LEFT;


  // Ensure motor speed does not go too high
  u = constrain(u, -200, 200);  // Limit the speed to motor's maximum range
  u_LEFT = constrain(u_LEFT, -200, 200);



  setMotor(u, PWM, IN1, IN2);
  setMotor(u_LEFT, PWM_LEFT, IN3_LEFT, IN4_LEFT);




  // Print data: ax, ay, az, gx, gy, gz, q0-q3, v1, v1_LEFT
  // Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az); Serial.print(",");
  // Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.print(gz); Serial.print(",");
  // Serial.print(q0); Serial.print(","); Serial.print(q1); Serial.print(","); Serial.print(q2); Serial.print(","); Serial.print(q3); Serial.print(",");
  
  
  Serial.print(v1Filt); Serial.print(","); Serial.println(v1Filt_LEFT);

  delay(50);  // 20 Hz
}

void setMotor(float u, int pwm, int in1, int in2) {
  int dir = u >= 0 ? 1 : -1;
  int pwr = min(abs(int(u)), 255);
  analogWrite(pwm, pwr);
  digitalWrite(in1, dir == 1 ? HIGH : LOW);
  digitalWrite(in2, dir == 1 ? LOW : HIGH);
}

void readEncoder() {
  pos_i++;
  long currT = micros();
  velocity_i = 1.0 / ((currT - prevT_i) / 1.0e6);
  prevT_i = currT;
}

void readEncoder_LEFT() {
  pos_i_LEFT++;
  long currT = micros();
  velocity_i_LEFT = 1.0 / ((currT - prevT_i_LEFT) / 1.0e6);
  prevT_i_LEFT = currT;
}

void updateOrientation(float gx, float gy, float gz, float ax, float ay, float az) {
  float dt = 0.05;
  float norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm; ay /= norm; az /= norm;

  float qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);

  q0 += qDot1 * dt; q1 += qDot2 * dt;
  q2 += qDot3 * dt; q3 += qDot4 * dt;

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}
