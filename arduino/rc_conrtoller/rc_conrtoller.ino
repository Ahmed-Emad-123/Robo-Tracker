#include "ros.h"
#include <std_msgs/Int16MultiArray.h>

// Motors Connections
#define enA 3 
#define in1 9
#define in2 10
#define in3 11
#define in4 12
#define enB 5

// Motor Definitions
const int Speed = 150;

// ROS Definitions
ros::NodeHandle nh;
int dir = 0;
int vel = 0;

int led = 13;

void cmd_Cb(const std_msgs::Int16MultiArray& msg){
  
  // get direction and velocity from array:::
  dir = msg.data[1];
  vel = msg.data[0];

  // if left:
  if(dir == -1){
    digitalWrite(led, HIGH);
    backward(vel);  
  }

  // if right:
  else if(dir == 1){
    digitalWrite(led, LOW);
     forward(vel);
  }

  // if stop:
  else{
    Stop();
  }
  
}
  
ros::Subscriber <std_msgs::Int16MultiArray> cmd_sub("/cmd_vel", &cmd_Cb);

void setup() {
  // Setup ROS Buad Rate
  nh.getHardware()->setBaud(9600);
  
  // ROS Configuration
  nh.initNode();
  nh.subscribe(cmd_sub);

    // Motors Pin Configurations
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(enB,OUTPUT);

  // Initialy Stop Motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);  
  pinMode(led, OUTPUT);
}

void loop() {

  nh.spinOnce();
  delay(15);

}



void left(int vel) {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, vel);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, vel);
  
}


void right(int vel) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, vel);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, vel);
}

void forward(int vel) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, vel);
  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, vel);
}

void backward(int vel) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, vel);
  
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, vel);
}

void Stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
