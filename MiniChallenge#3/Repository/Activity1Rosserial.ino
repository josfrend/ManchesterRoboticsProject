#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#define pwm_pin 2

ros::NodeHandle  nh;
#pwm_value initialization
int pwm_value = 255;

std_msgs::Int16 str_msg;
ros::Publisher chatter("chatter", &str_msg);

#Funcion de callback
void messageCB(const std_msgs::Int16& toogle_msg){
  pwm_value = toogle_msg.data;
}

ros::Subscriber <std_msgs::Int16> sub("cmd_pwm", &messageCB);


void setup()
{
  #Pins setup
  pinMode(pwm_pin, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  
  #Handler setup
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  #Publish received value to check behaviour
  str_msg.data = pwm_value;
  chatter.publish( &str_msg);
  
  #Output of pwm value in clockwise direction 
  if(pwm_value > 255 ){
    analogWrite(3,255);
    analogWrite(4,0);
    pwm_value = pwm_value - 255; #Received value minus offset
    
  }
  #Output of pwm value in counter clockwise direction
  else if(pwm_value <= 255){
    analogWrite(4,255);
    analogWrite(3,0);
    pwm_value = -1 *(pwm_value - 510); #Received value minus entire resolution to get the actual magnitude
  }
  analogWrite(pwm_pin, pwm_value);
  nh.spinOnce();
  delay(100);
}
