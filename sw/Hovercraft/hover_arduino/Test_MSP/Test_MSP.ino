#include <MSP.h>

MSP msp;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(500000);
  msp.begin(Serial2);
  msp_motor_t motors;

  for(int i=0; i<MSP_MAX_SUPPORTED_MOTORS; i++)
    motors.motor[i] = 1000;

  msp.command(MSP_SET_MOTOR, &motors, sizeof(motors), false);

  delay(2000);
  motors.motor[0] = 1300;
  motors.motor[1] = 1300;
  motors.motor[2] = 1300;
  motors.motor[3] = 1300;
  motors.motor[4] = 1300;
  motors.motor[5] = 1300;
  msp.command(MSP_SET_MOTOR, &motors, sizeof(motors), false);
  
}

void loop()
{
  msp_rc_t rc;
  
  msp_attitude_t attitude;
  if (msp.request(MSP_ATTITUDE, &attitude, sizeof(attitude))) {
    
    float roll     = (float)attitude.roll;
    float pitch    = (float)attitude.pitch;
    float yaw      = (float)attitude.yaw;

    Serial.println("yaw");
    Serial.println(yaw);
    delay(100);
    
  }
}
