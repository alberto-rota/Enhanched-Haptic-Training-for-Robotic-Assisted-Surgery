// #include <Arduino.h>

// const int INSERTION_PIN = A3;
// const int PITCH_PIN = A4;
// const int YAW_PIN = A5;

// void setup() {
//   Serial.begin(9600);
//   pinMode(INSERTION_PIN,INPUT_PULLUP);
//   pinMode(PITCH_PIN,INPUT);
//   pinMode(YAW_PIN,INPUT);
// }

// void loop() {
//   double pitch = map(analogRead(PITCH_PIN),0,1023,0,100);
//   double yaw = map(analogRead(YAW_PIN),0,1023,0,100);
//   double insertion = map(analogRead(INSERTION_PIN),0,1023,0,100);
//   Serial.print(pitch);Serial.print(",");
//   Serial.print(yaw);Serial.print(",");
//   Serial.println(insertion);

// }

/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}