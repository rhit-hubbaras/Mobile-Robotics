/*   
HC05 - Bluetooth AT-Command mode  
modified on 10 Feb 2019 
by Saeed Hosseini 
https://electropeak.com/learn/ 
*/ 
#include <SoftwareSerial.h> 
SoftwareSerial MyBlue(0, 1); // RX | TX 
int flag = 0; 
int LED = 5; 
void setup() 
{   
  Serial.begin(115200); 
  MyBlue.begin(115200); 
  pinMode(LED, OUTPUT); 
  pinMode(0, INPUT);
  pinMode(1, OUTPUT);
  Serial.println("Ready to connect\nDefualt password is 1234 or 000"); 
} 
void loop() 
{ 
  //Serial.println(Serial.available());
  if (Serial.available()>0) 
    flag = Serial.read(); 
    //Serial.println(flag);
  if (flag == 49) 
  { 
    digitalWrite(LED, HIGH); 
    //Serial.println("LED On"); 
  } 
  else if (flag == 48) 
  { 
    digitalWrite(LED, LOW); 
    //Serial.println("LED Off"); 
  } 
  //Serial.println(flag);
} 
