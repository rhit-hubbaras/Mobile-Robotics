//ECE435 Mobile Robotics - Moravec Final Project GUI

import controlP5.*;  //importing ControlP5 library
import processing.serial.*;

// variables
Serial port;
ControlP5 cp5;
String textfield = "";
int robotx = 5;
int roboty = 5;
int grid_length = 500;
int grid_width = 500;
int gridposx = 100;
int gridposy = 100;

void setup() {
  
  size(1000,800);
  
  printArray(Serial.list()); //prints all available serial ports
  
  //port = new Serial(this,"COM5",115200);
  
  cp5 = new ControlP5(this);
  
  cp5.addButton("LED_test")
    .setPosition(700,200)
    .setSize(100,80);
    
  cp5.addButton("LED_Off")
    .setPosition(700,400)
    .setSize(100,80);

  cp5.addTextfield("Enter Topological Path Plan")
     .setPosition(700,600)
     .setSize(200,40);
}

void draw() {
  
  rect(gridposx,gridposy,grid_width,grid_length);
  fill(0);
  rect(robotx+gridposx,roboty+gridposy,50,50);
  fill(153);
  text("Topological Mapping GUI", 500,50);
  
}

void LED_Test(){
  port.write("49");
}

void LED_Off(){
  port.write("48");
}

public void Robot_X(String text) {
  println("received input value: " + text);
  //port.write("text");
}
