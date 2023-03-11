#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#define pwm_pin 2
#define ENCA 18
#define ENCB 19
#define ENCPULSES 13

ros::NodeHandle  nh;
int pwm_value = 100;
bool direction;
volatile int pulse_count;
int span = 10;
long previous_time = 0;
long current_time = 0;


float rpm = 0;
float angular_velocity = 0;
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;


std_msgs::Int16 str_msg;
std_msgs::Float32 omega_msg;
ros::Publisher chatter("omega", &str_msg);
ros::Publisher chatter2("omega2", &omega_msg);

void messageCB(const std_msgs::Int16& toogle_msg){
  pwm_value = toogle_msg.data;
}

ros::Subscriber <std_msgs::Int16> sub("cmd_pwm", &messageCB);

char hello[13] = "hello world!";

void setup()
{
  pinMode(pwm_pin, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(chatter2);
  nh.subscribe(sub);
  attachInterrupt(digitalPinToInterrupt(ENCA),ENC_read,RISING);
}

void loop()
{
  str_msg.data = pwm_value;
  if(pwm_value > 255){
    analogWrite(3,255);
    analogWrite(4,0);
    pwm_value = pwm_value - 255;
  }else if(pwm_value <= 255){
    analogWrite(4,255);
    analogWrite(3,0);
    pwm_value = -1 *(pwm_value - 510);
  }
  current_time = millis();
  if(current_time-previous_time > span){
    previous_time = current_time;
    rpm = (float)(pulse_count * 60 / ENCPULSES);
    angular_velocity = rpm * rpm_to_radians;

    pulse_count = 0;
  }
  omega_msg.data = angular_velocity;
  chatter2.publish(&omega_msg);
  analogWrite(pwm_pin, pwm_value);
  nh.spinOnce();
  delay(10);
}

void ENC_read(){
  int b = digitalRead(ENCB);
  if(b == LOW){
    direction = false;
  } 
  else{
    direction = true;
  }

  if(direction){
    pulse_count++;
  }
  else{
    pulse_count--;
  }
}

