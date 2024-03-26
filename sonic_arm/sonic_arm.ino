#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SERVO_PIN 9 // wire pwm servo lead to pin 9

Adafruit_MPU6050 mpu;

const float LOWEST_POSITION = 30.0; // lowest angle to go (extension)
const float HIGHEST_POSITION = 75.0; // highest angle to go (resting)
const uint8_t ANGLE_THRESHOLD = 2; // min angle change for collision not to be detected
const unsigned long COLLISION_TIME_THRESHOLD = 1500; // time limit for collision detection

// ROS configurations
ros::NodeHandle nh;
char topic[] = "nudge";

// motor choice
Servo armServo;

// accelerometer readings
float acc_x, acc_y, acc_z;

// states
enum State {IDLE, LOWER, TAP, RETRACT};
State state;

// using MPU6050 accelerometer

// function gets called when message is received on topic "nudge"
void nudgeCallback(const std_msgs::UInt16& cmd_msg) {
  // only start nudging if not already in progress
  if (state == IDLE){
    state = LOWER;
  }
}

// subscribe to topic "nudge", redirect to nudgeCallback on message receipt
ros::Subscriber<std_msgs::UInt16> nudgeSub(topic, &nudgeCallback);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  nh.initNode();
  nh.subscribe(nudgeSub); // subscribing to topic

  armServo.attach(9); // Attach servo to pin 9

  int mpu_init_counter = 0;
  while (!mpu.begin() && mpu_init_counter < 4) {
    Serial.println("Failed to find MPU6050 chip");
    mpu_init_counter +=1;
    delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");

  State state = IDLE;
  float last_angle = 180.0; // setting to high value so angle threshold is met on start of nudge
  unsigned long last_collision_time = 0;

  delay(100);
}


void loop() {
  nh.spinOnce(); // ROS processing

  // get accelerometer readings and calculate rotation angle
  sensors_event_t a;
  mpu.getEvent(&a, NULL, NULL); // only reading acceleration data
  acc_x = a.acceleration.x;
  acc_y = a.acceleration.y;
  acc_z = a.acceleration.z;
  acceleration = acc_y; // using y measurement as acceleration (depends on accelerometer placement)

  float rotation_angle = asin(acceleration / 9.81) * (180.0 / PI); // convert radians to degrees
  
  // Filter angle
  switch(state)
  {
    case LOWER:
      myservo.writeMicroseconds(2200); // unwind
      if (abs(rotation_angle - last_angle) > ANGLE_THRESHOLD) {
        last_collision_time = millis();
        last_angle = rotation_angle;
      } else {
        // check if time threshold elapsed without angle change or if reached lowest position
        if ((millis() - lastCollisionTime > COLLISION_TIME_THRESHOLD) || (rotation_angle < LOWEST_POSITION)) {
          // contact made -> move into tap mode
          myservo.writeMicroseconds(1500); // stop the arm
          state = TAP;
        } 
      }

    case TAP:
      if (tap_sequence >= 24){ // do 6 taps
        tap_sequence = 0;
        state = RETRACT;
      } else {
        if (tap_sequence % 4 == 0) {
          myservo.writeMicroseconds(900); // up
        }
        if (tap_sequence % 4 == 1 || tap_sequence % 4 == 3) {
          myservo.writeMicroseconds(1500); // stop
        }
        if (tap_sequence % 4 == 2) {
          myservo.writeMicroseconds(2200); // down
        }
        tap_sequence ++;
      }   

    case RETRACT:
      myservo.writeMicroseconds(900); // wind back up
      if rotation_angle > HIGHEST_POSITION {
        myservo.writeMicroseconds(1500); // stop after reaching limit
        state = IDLE;
      }
    case default: // same as IDLE, just waiting for signal
      continue;

    delay(100);
  }

}
