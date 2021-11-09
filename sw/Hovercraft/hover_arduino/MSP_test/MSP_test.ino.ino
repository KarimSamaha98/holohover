#include <MSP.h>
#include <HardwareSerial.h>

MSP msp;


msp_motor_t motors;  // Motor states



float roll;
float pitch;
float yaw;

void setup() {
  // Connection to Drone
  // Make sure that betaflight is configured to the same baud rate
  Serial2.begin(115200);
  msp.begin(Serial2);

  // Default motor state : OFF
  // 1000 = 0%, 2000 = 100%
  for(int i=0; i<MSP_MAX_SUPPORTED_MOTORS; i++)
    motors.motor[i] = 1000;

  Serial.begin(115200);


}

void loop() {
  // put your main code here, to run repeatedly:
  msp_attitude_t attitude; // Drone state
  msp_raw_imu_t imu;
  Serial.println("Original Attitude");
  Serial.println((float)attitude);
  Serial.println("Requesting IMU info...");
  answer = msp.request(MSP_ATTITUDE, &attitude, sizeof(msp_attitude_t));
  msp.request(MSP_RAW_IMU, &imu, sizeof(msp_raw_imu_t));
  Serial.println("IMU info received");

  delay(1000);
  roll  = ((float)attitude.roll);
  pitch = ((float)attitude.pitch);
  yaw   = ((float)attitude.yaw);
  Serial.println(roll);
  Serial.println(pitch);
  Serial.println(yaw);

//   motors.motor[1] = 2000;
//   motors.motor[2] = 2000;
//   motors.motor[3] = 2000;
//   motors.motor[4] = 2000;
//   motors.motor[5] = 2000;
//   motors.motor[6] = 2000;
//   //msp.command(MSP_SET_MOTOR, &motors, sizeof(motors), false);  
//   Serial.println("Sent motor commands");
   

  delay(500);
}
