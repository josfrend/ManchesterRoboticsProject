#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#define pwm_pin 2

ros::NodeHandle  nh;
int pwm_value = 100;

std_msgs::Int16 str_msg;
ros::Publisher chatter("chatter", &str_msg);

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
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  str_msg.data = pwm_value;
  chatter.publish( &str_msg);
  if(pwm_value > 255 ){
    analogWrite(3,255);
    analogWrite(4,0);
    pwm_value = pwm_value - 255;
  }else if(pwm_value <= 255){
    analogWrite(4,255);
    analogWrite(3,0);
    pwm_value = -1 *(pwm_value - 510);
  }
  analogWrite(pwm_pin, pwm_value);
  nh.spinOnce();
  delay(100);
}
