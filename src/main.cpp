#include <Arduino.h>
#include <HardwareSerial.h>
#include <Ultrasonic.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>

#define TRIG 13
#define ECHO 12

#define BIN1 7
#define BIN2 6
#define PWMB 5
#define STBY 8
#define PWMA 11
#define AIN2 10
#define AIN1 9

#define ENCODER_HOLE 20

#define VALUE 20

//definition of variable
unsigned long loop_start = 0;

unsigned long loop_dt = 0;
unsigned long range_timer;

volatile unsigned long path_r = 0;
volatile unsigned long path_l = 0;

volatile float time_l = 0;
volatile float time_l_last = 0;
volatile float time_l_array[VALUE];

volatile long encoder_l_pos = 0;
volatile long encoder_r_pos = 0;

float vel;
float ang;
float vl;
float vr;
float time_middle_l = 0;
float time_middle_r = 0;

int i, j;

volatile bool direction_l = true;
volatile bool direction_r = true;

volatile float time_r = 0;
volatile float time_r_last = 0;
volatile float time_r_array[VALUE];

float time_l_avg = 0;
float time_r_avg = 0;
float time_l_avg0 = 0;
float time_r_avg0 = 0;

float linear_vel = 0;
float angular_vel = 0;

int encoder[2] = {0, 0};

void left_it()
{
  if (direction_l == true)
  {
    encoder_l_pos += 1;
  }
  else
  {
    encoder_l_pos -= 1;
  }
}

void right_it()
{
  if (direction_r == true)
  {
    encoder_r_pos += 1;
  }
  else
  {
    encoder_r_pos -= 1;
  }
}

void motor_left(int power)
{
  digitalWrite(STBY, HIGH);
  if (power > 0)
  {
    direction_l = true;
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, power);
  }
  else if (power < 0)
  {
    direction_l = false;
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, abs(power));
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void motor_right(int power)
{
  digitalWrite(STBY, HIGH);
  if (power > 0)
  {
    direction_r = true;
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, power);
  }
  else if (power < 0)
  {
    direction_r = false;
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(power));
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

Ultrasonic sensor = Ultrasonic(13, 12);

ros::NodeHandle node_handle;
std_msgs::Int16MultiArray publish_std;
sensor_msgs::Range publish_range;

void subscibe_twist(const geometry_msgs::Twist &msg){
  linear_vel = msg.linear.x;
  angular_vel = msg.angular.z;
  return;
}


ros::Subscriber<geometry_msgs::Twist> sub_twist("/cmd_vel", &subscibe_twist);
ros::Publisher pub_encoder("/encoder", &publish_std);
ros::Publisher pub_range("/range", &publish_range);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  for (int i = 5; i <= 11; i++)
  {
    pinMode(i, OUTPUT);
  }
  digitalWrite(STBY, LOW);
  sensor.setTimeout(40000UL);

  attachInterrupt(0, left_it, CHANGE);
  attachInterrupt(1, right_it, CHANGE);

  node_handle.initNode();
  node_handle.advertise(pub_encoder);
  node_handle.advertise(pub_range);
  node_handle.subscribe(sub_twist);

  motor_left(0);
  motor_right(0);
}

void loop()
{
  // put your main code here, to run repeatedly:
  publish_std.data[0] = encoder_l_pos;
  publish_std.data[1] = encoder_r_pos;


  publish_range.radiation_type = 0;
  publish_range.min_range = 0.02;
  publish_range.max_range = 2;
  publish_range.range = sensor.read() / 100.0;

  
  pub_encoder.publish(&publish_std);
  pub_range.publish(&publish_range);

  node_handle.spinOnce();
  delay(1);
}