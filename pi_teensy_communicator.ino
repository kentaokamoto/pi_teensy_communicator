#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/joy.h>

#include <Arduino_LSM9DS1.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define LED_PIN 13
#define MOTOR0_PIN 14
#define MOTOR1_PIN 15
#define MOTOR2_PIN 22
#define MOTOR3_PIN 23

const int PHz = 250;
const int Pcyc = 1000000/PHz;
const float pwmbit = 4095 /Pcyc;
bool motor_flag = true;

float calib_ax; float calib_ay; float calib_az;

float calib_gx; float calib_gy; float calib_gz;

float pwm_0 = 1100;
float pwm_1 = 1100;
float pwm_2 = 1100;
float pwm_3 = 1100;

rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_mag;

rcl_subscription_t joy_subscriber;
rcl_subscription_t pwm_subscriber;

rclc_executor_t executor;

rclc_support_t support;

rcl_allocator_t allocator;

rcl_node_t node;

rcl_timer_t timer;

sensor_msgs__msg__Imu msg_imu;
sensor_msgs__msg__MagneticField msg_mag;

sensor_msgs__msg__Joy joy_msg;
std_msgs__msg__Float32MultiArray pwm_msg;

//char debagc;
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void joy_callback(const void * msgin)
{
  const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;
  static int32_t buttons_memory[13];
  joy_msg.buttons.capacity = 26;
  joy_msg.buttons.data = buttons_memory;
  joy_msg.buttons.size = 13;
  
  static float axes_memory[6];
  joy_msg.axes.capacity = 24;
  joy_msg.axes.data = axes_memory;
  joy_msg.axes.size = 6;
  
  if(msg == NULL){ }else{
    if((msg->buttons.data[1])== 1){
      motor_flag = false;
    }
    if((msg->buttons.data[2])== 1){
      motor_flag = true;
    }
  }
}

void pwm_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  static float pwm_memory[4];
  pwm_msg.data.capacity = 20;
  pwm_msg.data.data = pwm_memory;
  pwm_msg.data.size = 4;
  if(msg==NULL){ }else{
    pwm_0 = msg->data.data[0];
    pwm_1 = msg->data.data[1];
    pwm_2 = msg->data.data[2];
    pwm_3 = msg->data.data[3];
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  Serial.println("debagc");
  if(motor_flag){
    analogWrite(MOTOR0_PIN, pwm_0*pwmbit);
    analogWrite(MOTOR1_PIN, pwm_1*pwmbit);
    analogWrite(MOTOR2_PIN, pwm_2*pwmbit);
    analogWrite(MOTOR3_PIN, pwm_3*pwmbit);
  }else{
    analogWrite(MOTOR0_PIN, 0*pwmbit);
    analogWrite(MOTOR1_PIN, 0*pwmbit);
    analogWrite(MOTOR2_PIN, 0*pwmbit);
    analogWrite(MOTOR3_PIN, 0*pwmbit);
  }
  
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    if(IMU.accelerationAvailable())
    {
      IMU.readAcceleration(ax,ay,az);
      ax = -(ax*9.81 + calib_ax);
      ay = ay*9.81 + calib_ay;
      az = -(az*9.81 + calib_az);
//      ax = ax*9.81;
//      ay = ay*9.81;
//      az = az*9.81;
    }
    if(IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(gx, gy, gz);
      gx = gx*3.142/180 + calib_gx;
      gy = -(gy*3.142/180 + calib_gy);
      gz = gz*3.142/180 + calib_gz;
    }
    if(IMU.magneticFieldAvailable())
    {
      IMU.readMagneticField(mx, my, mz);
      mx = mx;
      my = my;
      mz = mz;
    }
    Serial.println("debage");
    msg_imu.linear_acceleration.x = ax;
    msg_imu.linear_acceleration.y = ay;
    msg_imu.linear_acceleration.z = az;

    
    msg_imu.angular_velocity.x = gx;
    msg_imu.angular_velocity.y = gy;
    msg_imu.angular_velocity.z = gz;

    msg_mag.magnetic_field.x = mx;
    msg_mag.magnetic_field.y = my;
    msg_mag.magnetic_field.z = mz;
    Serial.println("debagf");
    RCSOFTCHECK(rcl_publish(&publisher_imu, (const void *) &msg_imu, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_mag, (const void *) &msg_mag, NULL));
  }
}

void setup() {
  set_microros_transports();
//  debagc = 's';
//  Serial.println("s");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
 
  /////////////////////////////////////calibration accele
  IMU.begin();

  delay(5000);
  float ax, ay, az, gx, gy, gz;
  float buffer_ax=0; float buffer_ay=0; float buffer_az=0;
  
  float buffer_gx=0; float buffer_gy=0; float buffer_gz=0;
  for(int i=0;i<10000;i++){
    if(IMU.accelerationAvailable())
    {
      IMU.readAcceleration(ax,ay,az);
      ax = ax*9.81;
      ay = ay*9.81;
      az = az*9.81;
    }
    if(IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(gx, gy, gz);
      gx = gx*3.142/180;
      gy = gy*3.142/180;
      gz = gz*3.142/180;
    }
    buffer_ax +=  ax;
    buffer_ay +=  ay;
    buffer_az +=  az;
    buffer_gx +=  gx;
    buffer_gy +=  gy;
    buffer_gz +=  gz;
    delay(0.1);
  }
  
  calib_ax = 0.0-buffer_ax/10000;
  calib_ay = 0.0-buffer_ay/10000;
  calib_az = 9.81-buffer_az/10000;

  calib_gx = 0.0-buffer_gx/10000;
  calib_gy = 0.0-buffer_gy/10000;
  calib_gz = 0.0-buffer_gz/10000;

//  if(calib_ax > 20){calib_ax = 0;}
//  if(calib_ay > 20){calib_ay = 0;}
//  if(calib_az > 20){calib_az = 0;}
  
  delay(3000);
  analogWriteResolution(12);
  delay(1000);
  analogWriteFrequency(MOTOR0_PIN, PHz);
  analogWriteFrequency(MOTOR1_PIN, PHz);
  analogWriteFrequency(MOTOR2_PIN, PHz);
  analogWriteFrequency(MOTOR3_PIN, PHz);
  
  analogWrite(MOTOR0_PIN, 2000*pwmbit);
  analogWrite(MOTOR1_PIN, 2000*pwmbit);
  analogWrite(MOTOR2_PIN, 2000*pwmbit);
  analogWrite(MOTOR3_PIN, 2000*pwmbit);
  
  delay(3000);
  
  analogWrite(MOTOR0_PIN, 1000*pwmbit);
  analogWrite(MOTOR1_PIN, 1000*pwmbit);
  analogWrite(MOTOR2_PIN, 1000*pwmbit);
  analogWrite(MOTOR3_PIN, 1000*pwmbit);
  delay(3000);
  ////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////
  allocator = rcl_get_default_allocator();
  

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // create node
  RCCHECK(rclc_node_init_default(&node, "pi_esp_communicator", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/copto/imu"));
    
  RCCHECK(rclc_publisher_init_default(
    &publisher_mag,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "/copto/mag"));

  // create subscriber
  joy_subscriber = rcl_get_zero_initialized_subscription();
  RCCHECK(rclc_subscription_init_default(
    &joy_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"));

    // create subscriber
  pwm_subscriber = rcl_get_zero_initialized_subscription();
  RCCHECK(rclc_subscription_init_default(
    &pwm_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/copto/pwm"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

//  debagc = 'f';
//  Serial.println("f");
  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &joy_subscriber, &joy_msg, &joy_callback, ON_NEW_DATA));  
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &pwm_callback, ON_NEW_DATA));  
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin(&executor));

//  rclc_exector_fini(&executor);
//  rcl_publisher_fini(&publisher_imu, &node);
//  rcl_timer_fini(&timer);
//  rcl_subscription_fini(&joy_subscriber,&node);
//  rcl_subscription_fini(&pwm_subscriber,&node);
//  rcl_node_fini(&node);
//  rclc_support_fini(&support);
//  sensor_msgs__msg__Joy__fini(&joy_msg);
//  std_msgs__msg__Float32MultiArray__fini(&pwm_msg);
}