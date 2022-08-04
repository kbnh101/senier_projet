/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
int red = 10;
int green = 8;

String LED;

void messageCb( const std_msgs::String& msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  LED = msg.data;
  if(LED == "red")
  {
    digitalWrite(10, HIGH-digitalRead(10));
  }
  if(LED == "green")
  {
    digitalWrite(8, HIGH-digitalRead(8));
  }
}

ros::Subscriber<std_msgs::String> sub("state", messageCb );



void setup()
{
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(500);
}
