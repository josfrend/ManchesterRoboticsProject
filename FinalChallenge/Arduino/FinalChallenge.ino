#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#define pwm_pin 2
#define ENCA 18
#define ENCB 19
#define ENCPULSES 13

ros::NodeHandle  nh;
int pwm_value = 0;
bool direction;
volatile int pulse_count;
int span = 12;
long previous_time = 0;
long current_time = 0;


float rpm = 0;
float angular_velocity = 0;
const float rpm_to_radians = 0.10471975512;
const float rad_to_deg = 57.29578;
int movingAverage[2];
int mCount = 0;

std_msgs::Int16 str_msg;
std_msgs::Float32 omega_msg;

ros::Publisher chatter("motor_output", &omega_msg);

void messageCB(const std_msgs::Float32& toogle_msg){
  pwm_value = toogle_msg.data * 255.0;
}

ros::Subscriber <std_msgs::Float32> sub("motor_input", &messageCB);

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
  nh.subscribe(sub);
  attachInterrupt(digitalPinToInterrupt(ENCA),ENC_read,RISING);
}

void loop()
{
  str_msg.data = pwm_value;
  if(pwm_value > 0){
    analogWrite(3,0);
    analogWrite(4,255);
    pwm_value = pwm_value;
  }else if(pwm_value <= 0){
    analogWrite(3,255);
    analogWrite(4,0);
    pwm_value = -1 *(pwm_value);
  }
  current_time = millis();
  if(current_time-previous_time > span){
    previous_time = current_time;
    rpm = (float)(pulse_count * 60 / ENCPULSES);
    angular_velocity = rpm * rpm_to_radians;

    pulse_count = 0;
    /*
    if(mCount == 0){
      for(int i = 0; i <2; i++){
        movingAverage[i] = angular_velocity;
      }
      mCount++;
    }else{
      for(int i=1;i<2;i++){
        movingAverage[i-1] = movingAverage[i];
      }
      movingAverage[1] = angular_velocity;
    }

    angular_velocity = 0;
    
    for(int j = 0; j <2 ; j++){
      angular_velocity += movingAverage[j]; 
    }
    angular_velocity = angular_velocity/2;
    */
  }

  
  
 
  omega_msg.data = angular_velocity;
  chatter.publish(&omega_msg);
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

