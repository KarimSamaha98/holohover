/*
 * Arduino code for the hovercraft
 * 
 * Runs on a ESP32 and echos commands from the ground station coming in over WIFI to the betaflight board over a serial line
 * 
 */


// Google protobuf communication structure
#include "hovercraft.pb.h"  

// nanopb (protobuf for embedded systems)
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "WiFi.h"
#include "AsyncUDP.h"

#include <MSP.h>
#include <HardwareSerial.h>

/*
 * Enter network name and wifi password here
 */
const char * ssid = "Karim";
const char * password = "karimsamaha";

const int udpPort = 3333;

AsyncUDP udp;
MSP msp;

#define MOTOR_A_1  0
#define MOTOR_A_2  1 
#define MOTOR_B_1  2 
#define MOTOR_B_2  3 
#define MOTOR_C_1  4 
#define MOTOR_C_2  5 

msp_attitude_t attitude; // Drone state
msp_raw_imu_t imu;
msp_motor_t motors;  // Motor states

bool update_motors = false; //Motors off


unsigned long prev_display_time  = 0;
int count = 0;

unsigned long period = 10000; // us
unsigned long prev_time  = 0;


void setup()
{
  // Connection to Drone
  // Make sure that betaflight is configured to the same baud rate
  Serial2.begin(500000);
  msp.begin(Serial2);

  // Default motor state : OFF
  // 1000 = 0%, 2000 = 100%
  for(int i=0; i<MSP_MAX_SUPPORTED_MOTORS; i++)
    motors.motor[i] = 1000;

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  if(udp.listen(udpPort)) 
  {
    Serial.print("Drone Listening on IP: ");
    Serial.println(WiFi.localIP());
  }

  prev_display_time = millis();
  prev_time = micros();
}


void loop()
{
  uint8_t buffer[128];
  DroneState state = DroneState_init_zero;
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Get current drone state
  Serial.println("Requesting IMU info...");
  msp.request(MSP_ATTITUDE, &attitude, sizeof(attitude));
  msp.request(MSP_RAW_IMU, &imu, sizeof(imu));
  Serial.println("IMU info received");

  
  state.roll  = ((float)attitude.roll);
  state.pitch = ((float)attitude.pitch);
  state.yaw   = ((float)attitude.yaw);

  state.acc_x  = imu.acc[0];
  state.acc_y  = imu.acc[1];
  state.acc_z  = imu.acc[2];
  state.gyro_x = imu.gyro[0];
  state.gyro_y = imu.gyro[1];
  state.gyro_z = imu.gyro[2];
  state.mag_x  = imu.mag[0];
  state.mag_y  = imu.mag[1];
  state.mag_z  = imu.mag[2];

  Serial.println(state.acc_x);
  bool status = pb_encode(&stream, DroneState_fields, &state);
  if (!status)
  {
    Serial.println("Failed to encode");
    return;
  }

  // Send broadcast to PC
  Serial.println("Sending broadcast to PC...");
  udp.broadcast(buffer, stream.bytes_written);
  Serial.println("Broadcast sent");


  // Code executed whenever a packet is received
  udp.onPacket([](AsyncUDPPacket packet) 
  {
    // Decode packet
    Motors motors_in = Motors_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(packet.data(), packet.length());      
    bool status = pb_decode(&stream, Motors_fields, &motors_in);

    if (!status)
    {
      Serial.println("ERROR - could not decode packet");
    } else
    {
      // Schedule motor speed transmission to drone
      Serial.println("Updating motor value");
      motors.motor[MOTOR_A_1] = constrain(motors_in.motor_A_1, 1000, 2000);
      motors.motor[MOTOR_A_2] = constrain(motors_in.motor_A_2, 1000, 2000);
      motors.motor[MOTOR_B_1] = constrain(motors_in.motor_B_1, 1000, 2000);
      motors.motor[MOTOR_B_2] = constrain(motors_in.motor_B_2, 1000, 2000);
      motors.motor[MOTOR_C_1] = constrain(motors_in.motor_C_1, 1000, 2000);
      motors.motor[MOTOR_C_2] = constrain(motors_in.motor_C_2, 1000, 2000);
      
      update_motors = true;
    };
  });

  
  if (update_motors)
  {
      // Send command to FC
      msp.command(MSP_SET_MOTOR, &motors, sizeof(motors), false); 
      Serial.println("Sending motor commands to FC");
      update_motors = false;  
  }

  // Sleep to the next period  
  count = count + 1;
  if (count > 100)
  {
    Serial.print("Loop time : ");    
    Serial.println((float)(millis() - prev_display_time ) / 100.0);
    prev_display_time = millis();
    count = 0;
  }

  delayMicroseconds((unsigned int)(prev_time + period - micros()));
  prev_time = micros();
}
