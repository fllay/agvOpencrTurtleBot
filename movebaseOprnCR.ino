/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>

//#define ACCEL_FACTOR                      8.0/32768.0*9.80665   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
//#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]
//#define MAG_FACTOR                        15e-8

#define ACCEL_FACTOR                      0.000598550415   // (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
                                                           //                                             Scale : +- 16384
#define GYRO_FACTOR                       0.0010642        // (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
                                                           //                                             Scale : +- 16.4[deg/s]

#define MAG_FACTOR                        15e-8

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif    




#define BAUDRATE  1000000
#define DXL_ID_LEFT    1
#define DXL_ID_RIGHT    2

char imu_frame_id[30];
char mag_frame_id[30];

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "lino_velocities.h"
#include "geometry_msgs/Twist.h"
#include "lino_imu.h"
#include "Kinematics.h"

#include <IMU.h>

#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
#define MAX_RPM 55             // motor's maximum RPM
#define WHEEL_DIAMETER 0.066       // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.16  // distance between left and right wheels 0.800
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define USE_MPU9250_IMU


#define COMMAND_RATE 30
#define ODOM_RATE 30

cIMU    IMU;

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;

int currentLeftWheelRPM;
int currentRightWheelRPM;
std_msgs::Int32 rpmLeft;
std_msgs::Int32 rpmRight;

int led_state = 0;

void commandCallback(const geometry_msgs::Twist& cmd_msg);
void moveBase();


void waitForSerialLink(bool isConnected);


ros::NodeHandle nh;


geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;




ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);

/*lino_msgs::Imu raw_IMUmsg;
ros::Publisher raw_IMUpub("raw_imu", &raw_IMUmsg);
*/

/*sensor_msgs::Imu raw_data_imu;
ros::Publisher raw_IMUdata_pub("/imu/data_raw", &raw_data_imu);
*/
//***** IMU Pub declear ****

sensor_msgs::Imu imu;
ros::Publisher imu_pub("/imu/data", &imu);


sensor_msgs::MagneticField raw_data_mag;
ros::Publisher raw_mag_data_pub("/imu/mag", &raw_data_mag);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

ros::Publisher rpmLeft_pub("rpmLeft", &rpmLeft);

ros::Publisher rpmRight_pub("rpmRight", &rpmRight);



int led_pin = 13;
unsigned long prev_update_time_m1 = 0;
unsigned long prev_update_time_m2 = 0;

unsigned long pulseRPM1 = 0;
unsigned long pulseRPM2 = 0;

unsigned long prev_update_time_M1 = 0;
unsigned long prev_update_time_M2 = 0;

unsigned long prev_encoder_ticks_M1 = 0;
unsigned long prev_encoder_ticks_M2 = 0;
int counts_per_rev_ = 15;



DynamixelWorkbench dxl_wb_left;
DynamixelWorkbench dxl_wb_right;



void setLeftRPM(int rpm){
      int32_t *get_data;
   /* if(rpm < 0){
        //M1_dir = 1;
        currentLeftWheelRPM = -1*getRPM1();
    } else {
        //M1_dir = 0;
        currentLeftWheelRPM = getRPM1();
    }*/
    
     get_data = dxl_wb_left.syncRead("Present_Velocity");

      int32_t speed = get_data[0];
      double rpm1 = (double) speed*0.229;
      //Serial.print("PresVel Left: ");
      //Serial.print(rpm1);

      //Serial.println("");
      currentLeftWheelRPM = (int) rpm1;
      dxl_wb_left.goalSpeed(DXL_ID_LEFT, rpm*100/22.9);
  
    
       
}

void setRightRPM(int rpm){
    int32_t *get_data;
    
  /*     if(rpm < 0){
        //M2_dir = 1;
        currentRightWheelRPM = -1*getRPM2();
    } else {
        //M2_dir = 0;
        currentRightWheelRPM = getRPM2();
    }*/

    get_data = dxl_wb_right.syncRead("Present_Velocity");
    int32_t speed = get_data[0];
    double rpm1 = (double) speed*0.229;
    //Serial.print("PresVel Right: ");
    //Serial.print(rpm1);
    //Serial.println("");
    currentRightWheelRPM = (int) rpm1;
    dxl_wb_right.goalSpeed(DXL_ID_RIGHT, rpm*100/22.9);
}



void setup() 
{
  
  pinMode(led_pin, OUTPUT);
  pinMode(BDPIN_LED_USER_1, OUTPUT);
  
  //Serial.begin(57600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_sub);
  nh.advertise(raw_vel_pub);
  
  /*nh.advertise(raw_IMUpub);
  nh.advertise(raw_IMUdata_pub);*/
  
  nh.advertise(imu_pub);
  nh.advertise(raw_mag_data_pub);
  
  nh.advertise(rpmLeft_pub);
  nh.advertise(rpmRight_pub);
  // while(!Serial); // If this line is activated, you need to open Serial Terminal.

   sprintf(imu_frame_id, "imu_link");
   sprintf(mag_frame_id, "imu_link");


  
  
  IMU.begin();

  /*  IMU.SEN.gyro_cali_start();
  while(!IMU.SEN.gyro_cali_get_done())
  {
    IMU.update();
  }*/


  calibrationGyro();


 dxl_wb_left.begin(DEVICE_NAME, BAUDRATE);
  dxl_wb_right.begin(DEVICE_NAME, BAUDRATE);
  dxl_wb_left.ping(DXL_ID_LEFT);
  dxl_wb_right.ping(DXL_ID_RIGHT);
  dxl_wb_left.addSyncRead("Present_Velocity");
  dxl_wb_right.addSyncRead("Present_Velocity");

  dxl_wb_left.wheelMode(DXL_ID_LEFT);
  dxl_wb_right.wheelMode(DXL_ID_RIGHT);
  
  

  
}



void calibrationGyro()
{
  uint32_t pre_time;
  uint32_t t_time;

  //const uint8_t led_ros_connect = 3;

  IMU.SEN.gyro_cali_start();
  
  t_time   = millis();
  pre_time = millis();

  while(!IMU.SEN.gyro_cali_get_done())
  {
    IMU.update();

    if (millis()-pre_time > 20000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      //setLedToggle(led_ros_connect);
      digitalWrite(BDPIN_LED_USER_1, !digitalRead(BDPIN_LED_USER_1));
    }
  }

  digitalWrite(BDPIN_LED_USER_1,HIGH);

  
}



/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}


void loop() 
{




  Kinematics::velocities current_vel;
  static unsigned long prev_IMUtime = 0;
  static unsigned long prev_control_time = 0;
  static unsigned long prev_odom_time = 0;

  
        //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)){   
    moveBase();
    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
        rpmLeft.data = currentLeftWheelRPM;
    rpmRight.data = currentRightWheelRPM;
    
    rpmLeft_pub.publish(&rpmLeft);
    rpmRight_pub.publish(&rpmRight);
  
    prev_control_time = millis();
  }

 

  if ((millis() - prev_odom_time) >= (1000 / ODOM_RATE)){ 

   int current_rpm1 = currentLeftWheelRPM; //rightWheel.getRPM();
    int current_rpm2 = currentRightWheelRPM; //leftWheel.getRPM();
    int current_rpm3 = 0;
    int current_rpm4 = 0;


    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    //current_vel = kinematics.getVelocities(50, 50, 0, 0);
    
    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
    //ros::Time stamp_now = nh.now();
    //raw_vel_msg.header.stamp = stamp_now;
    //publish raw_vel_msg
    
    raw_vel_pub.publish(&raw_vel_msg);
    prev_odom_time = millis();

         digitalWrite(led_pin, (led_state) ? HIGH : LOW);
            led_state = !led_state;

            

  }

  //calibrationGyro();
   IMU.update();


  /*  IMU.SEN.gyro_cali_start();
  while(!IMU.SEN.gyro_cali_get_done())
  {
    IMU.update();
  }*/
  

  if ((millis() - prev_IMUtime) >= (1000 / (50))){

          
                  
            /*raw_IMUmsg.linear_acceleration.x = IMU.SEN.accADC[0] * ACCEL_FACTOR;
            raw_IMUmsg.linear_acceleration.y = IMU.SEN.accADC[1] * ACCEL_FACTOR;
            raw_IMUmsg.linear_acceleration.z = IMU.SEN.accADC[2] * ACCEL_FACTOR;
   


           
            raw_IMUmsg.angular_velocity.x = IMU.SEN.gyroADC[0] * GYRO_FACTOR;
            raw_IMUmsg.angular_velocity.y = IMU.SEN.gyroADC[1] * GYRO_FACTOR;
            raw_IMUmsg.angular_velocity.z = IMU.SEN.gyroADC[2] * GYRO_FACTOR;

            
            raw_IMUmsg.magnetic_field.x = IMU.SEN.magADC[0] * MAG_FACTOR;
            raw_IMUmsg.magnetic_field.y = IMU.SEN.magADC[1] * MAG_FACTOR;
            raw_IMUmsg.magnetic_field.z = IMU.SEN.magADC[2] * MAG_FACTOR;
            raw_IMUpub.publish(&raw_IMUmsg);*/
    
   

            imu.angular_velocity.x = IMU.SEN.gyroADC[0] * GYRO_FACTOR;
            imu.angular_velocity.y = IMU.SEN.gyroADC[1] * GYRO_FACTOR;
            imu.angular_velocity.z = IMU.SEN.gyroADC[2] * GYRO_FACTOR;
            imu.angular_velocity_covariance[0] = 0.02;
            imu.angular_velocity_covariance[1] = 0;
            imu.angular_velocity_covariance[2] = 0;
            imu.angular_velocity_covariance[3] = 0;
            imu.angular_velocity_covariance[4] = 0.02;
            imu.angular_velocity_covariance[5] = 0;
            imu.angular_velocity_covariance[6] = 0;
            imu.angular_velocity_covariance[7] = 0;
            imu.angular_velocity_covariance[8] = 0.02;

            imu.linear_acceleration.x = IMU.SEN.accADC[0] * ACCEL_FACTOR;
            imu.linear_acceleration.y = IMU.SEN.accADC[1] * ACCEL_FACTOR;
            imu.linear_acceleration.z = IMU.SEN.accADC[2] * ACCEL_FACTOR;

            imu.linear_acceleration_covariance[0] = 0.04;
            imu.linear_acceleration_covariance[1] = 0;
            imu.linear_acceleration_covariance[2] = 0;
            imu.linear_acceleration_covariance[3] = 0;
            imu.linear_acceleration_covariance[4] = 0.04;
            imu.linear_acceleration_covariance[5] = 0;
            imu.linear_acceleration_covariance[6] = 0;
            imu.linear_acceleration_covariance[7] = 0;
            imu.linear_acceleration_covariance[8] = 0.04;

            imu.orientation.w = IMU.quat[0];
            imu.orientation.x = IMU.quat[1];
            imu.orientation.y = IMU.quat[2];
            imu.orientation.z = IMU.quat[3];

            imu.orientation_covariance[0] = 0.0025;
            imu.orientation_covariance[1] = 0;
            imu.orientation_covariance[2] = 0;
            imu.orientation_covariance[3] = 0;
            imu.orientation_covariance[4] = 0.0025;
            imu.orientation_covariance[5] = 0;
            imu.orientation_covariance[6] = 0;
            imu.orientation_covariance[7] = 0;
            imu.orientation_covariance[8] = 0.0025;     
            imu.header.stamp    = rosNow();
            imu.header.frame_id = "imu_link"; //imu_frame_id;
            imu_pub.publish(&imu);

            raw_data_mag.magnetic_field.x = IMU.SEN.magADC[0] * MAG_FACTOR;
            raw_data_mag.magnetic_field.y = IMU.SEN.magADC[1] * MAG_FACTOR;
            raw_data_mag.magnetic_field.z = IMU.SEN.magADC[2] * MAG_FACTOR;

            raw_data_mag.magnetic_field_covariance[0] = 0.0048;
            raw_data_mag.magnetic_field_covariance[1] = 0;
            raw_data_mag.magnetic_field_covariance[2] = 0;
            raw_data_mag.magnetic_field_covariance[3] = 0;
            raw_data_mag.magnetic_field_covariance[4] = 0.0048;
            raw_data_mag.magnetic_field_covariance[5] = 0;
            raw_data_mag.magnetic_field_covariance[6] = 0;
            raw_data_mag.magnetic_field_covariance[7] = 0;
            raw_data_mag.magnetic_field_covariance[8] = 0.0048;
            raw_data_mag.header.stamp    = rosNow();
            raw_data_mag.header.frame_id = "imu_link  `"; //mag_frame_id;

            raw_mag_data_pub.publish(&raw_data_mag);


           


            prev_IMUtime = millis();
        }


 
    nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
 
  
}




void stopBase()
{   

    setRightRPM(0);
  setLeftRPM(0);
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void moveBase(){
    
    
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
    setLeftRPM(req_rpm.motor1);
    setRightRPM(req_rpm.motor2);


      
   
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //char buffer[40];  
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;


    //moveBase();


    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
            
    g_prev_command_time = millis();
     
     //nh.spinOnce();

  // Wait the serial link time to process
  //waitForSerialLink(nh.connected());
}
