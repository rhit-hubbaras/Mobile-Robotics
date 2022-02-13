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
int gapwidth = 15;
int tilewidth = (grid_width-5*gapwidth)/4;
int wallwidth = 5;
int wallheight = tilewidth;
byte[][] map = new byte[4][4];
boolean slowManual = false;
int startx = -1;
int starty = -1;
int goalx = -1;
int goaly = -1;



void setup() {
  
  size(1000,700);
  
  println("test");
  printArray(Serial.list()); //prints all available serial ports
  
  //port = new Serial(this,"COM5",115200);
  
  cp5 = new ControlP5(this);
  
  //cp5.addButton("LED_test")
  //  .setPosition(700,200)
  //  .setSize(100,80);
    
  //cp5.addButton("LED_Off")
  //  .setPosition(700,400)
  //  .setSize(100,80);
  
  cp5.addButton("fwd")
     .setPosition(730,100)
     .setSize(25,25);
  cp5.addButton("bck")
     .setPosition(730,130)
     .setSize(25,25);
  cp5.addButton("lft")
     .setPosition(700,130)
     .setSize(25,25);
  cp5.addButton("rgt")
     .setPosition(760,130)
     .setSize(25,25);
  cp5.addButton("slo")
     .setPosition(760,100)
     .setSize(25,25);

  cp5.addTextfield("Topo_Path")
     .setPosition(700,200)
     .setSize(200,30)
     .setFocus(true);
     
  for(int i=0;i<3;i++){
    for(int j=0;j<4;j++){
      cp5.addButton("H"+str(i)+str(j))
        .setPosition(gridposx+(i+1)*(gapwidth+tilewidth),gridposy+(j+0.5)*(gapwidth+tilewidth))
        .setSize(gapwidth,gapwidth);
    }
  }
  
  for(int i=0;i<4;i++){
    for(int j=0;j<3;j++){
      cp5.addButton("V"+str(i)+str(j))
        .setPosition(gridposx+(i+0.5)*(gapwidth+tilewidth),gridposy+(j+1)*(gapwidth+tilewidth))
        .setSize(gapwidth,gapwidth);
    }
  }
  
  cp5.addButton("Send_Map")
     .setPosition(gridposx,gridposy+grid_width+10)
     .setSize(150,50);
     
  map[0][0] = 9;
  map[1][0] = 1;
  map[2][0] = 1;
  map[3][0] = 3;
  map[3][1] = 2;
  map[3][2] = 2;
  map[3][3] = 6;
  map[2][3] = 4;
  map[1][3] = 4;
  map[0][3] = 12;
  map[0][2] = 8;
  map[0][1] = 8;

  cp5.addTextfield("Start_Coord")
     .setPosition(450,615)
     .setSize(80,20)
     .setFocus(false);
     
  cp5.addTextfield("Goal_Coord")
     .setPosition(450,650)
     .setSize(80,20)
     .setFocus(false);
  
  cp5.addButton("STOP")
     .setPosition(820,100)
     .setSize(75,75);
}

void serialEvent (Serial port){
  String status = port.readStringUntil('\n');
  println(status);
}

void draw() {
  fill(200);
  rect(0,0,1000,1000);
  drawGrid();
  fill(153);
  textSize(13);
  text("Topological Mapping GUI", 500,50);
  textSize(16);
  text("Start Location: (" + str(startx) +", " +str(starty) +")",300,630);
  text("Goal Location: (" + str(goalx) +", " +str(goaly) +")",300,665);
  text("Status: " + "IDLE",100,80);
  if(slowManual){fill(180,0,0);}
  else{fill(153);}
  rect(700,160,85,15);
}

void drawGrid(){
  fill(153);
  rect(gridposx,gridposy,grid_width,grid_width);
  for(int i = 0; i<4; i++){
    for(int j = 0; j<4; j++){
      int tilex = gridposx+gapwidth+(gapwidth+tilewidth)*i;
      int tiley = gridposy+gapwidth+(gapwidth+tilewidth)*j;
      fill(200);
      rect(tilex,tiley,tilewidth,tilewidth);
      fill(0);
      if((map[i][j] & 1)>0){
        rect(tilex+(tilewidth-wallheight)/2,tiley,wallheight,wallwidth);
      }
      if((map[i][j] & 2)>0){
        rect(tilex+tilewidth-wallwidth,tiley+(tilewidth-wallheight)/2,wallwidth,wallheight);
      }
      if((map[i][j] & 4)>0){
        rect(tilex+(tilewidth-wallheight)/2,tiley+tilewidth-wallwidth,wallheight,wallwidth);
      }
      if((map[i][j] & 8)>0){
        rect(tilex,tiley+(tilewidth-wallheight)/2,wallwidth,wallheight);
      }
    }
  }
}

//void LED_Test(){
//  port.write("49");
//}

//void LED_Off(){
//  port.write("48");
//}

public void Topo_Path(String text) {
  println("received input value: " + text);
  port.write("t"+ text + "\n");
}

public void Start_Coord(String text) {
  println("start coords: " + text);
  startx = int(text.charAt(0))-48;
  starty = int(text.charAt(1))-48;
  port.write("s"+ text + "\n");
}
public void Goal_Coord(String text) {
  println("goal coords: " + text);
  goalx = int(text.charAt(0))-48;
  goaly = int(text.charAt(1))-48;
  port.write("f"+ text + "\n");
}

void STOP(){
  port.write('c');
}

void Send_Map(){
  port.write('m');
  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      port.write(map[i][j]);
    }
  }
}

void fwd(){port.write('d');if(slowManual){port.write(0);}else{port.write(4);}}
void bck(){port.write('d');if(slowManual){port.write(1);}else{port.write(5);}}
void lft(){port.write('d');if(slowManual){port.write(2);}else{port.write(6);}}
void rgt(){port.write('d');if(slowManual){port.write(3);}else{port.write(7);}}

void slo(){
  if(slowManual){slowManual = false;}
  else{slowManual = true;}
}

void H00(){if((map[0][0]&2)>0){map[0][0]-=2;map[1][0]-=8;}else{map[0][0]+=2;map[1][0]+=8;}}
void H10(){if((map[1][0]&2)>0){map[1][0]-=2;map[2][0]-=8;}else{map[1][0]+=2;map[2][0]+=8;}}
void H20(){if((map[2][0]&2)>0){map[2][0]-=2;map[3][0]-=8;}else{map[2][0]+=2;map[3][0]+=8;}}
void H01(){if((map[0][1]&2)>0){map[0][1]-=2;map[1][1]-=8;}else{map[0][1]+=2;map[1][1]+=8;}}
void H11(){if((map[1][1]&2)>0){map[1][1]-=2;map[2][1]-=8;}else{map[1][1]+=2;map[2][1]+=8;}}
void H21(){if((map[2][1]&2)>0){map[2][1]-=2;map[3][1]-=8;}else{map[2][1]+=2;map[3][1]+=8;}}
void H02(){if((map[0][2]&2)>0){map[0][2]-=2;map[1][2]-=8;}else{map[0][2]+=2;map[1][2]+=8;}}
void H12(){if((map[1][2]&2)>0){map[1][2]-=2;map[2][2]-=8;}else{map[1][2]+=2;map[2][2]+=8;}}
void H22(){if((map[2][2]&2)>0){map[2][2]-=2;map[3][2]-=8;}else{map[2][2]+=2;map[3][2]+=8;}}
void H03(){if((map[0][3]&2)>0){map[0][3]-=2;map[1][3]-=8;}else{map[0][3]+=2;map[1][3]+=8;}}
void H13(){if((map[1][3]&2)>0){map[1][3]-=2;map[2][3]-=8;}else{map[1][3]+=2;map[2][3]+=8;}}
void H23(){if((map[2][3]&2)>0){map[2][3]-=2;map[3][3]-=8;}else{map[2][3]+=2;map[3][3]+=8;}}

void V00(){if((map[0][0]&4)>0){map[0][0]-=4;map[0][1]-=1;}else{map[0][0]+=4;map[0][1]+=1;}}
void V01(){if((map[0][1]&4)>0){map[0][1]-=4;map[0][2]-=1;}else{map[0][1]+=4;map[0][2]+=1;}}
void V02(){if((map[0][2]&4)>0){map[0][2]-=4;map[0][3]-=1;}else{map[0][2]+=4;map[0][3]+=1;}}
void V10(){if((map[1][0]&4)>0){map[1][0]-=4;map[1][1]-=1;}else{map[1][0]+=4;map[1][1]+=1;}}
void V11(){if((map[1][1]&4)>0){map[1][1]-=4;map[1][2]-=1;}else{map[1][1]+=4;map[1][2]+=1;}}
void V12(){if((map[1][2]&4)>0){map[1][2]-=4;map[1][3]-=1;}else{map[1][2]+=4;map[1][3]+=1;}}
void V20(){if((map[2][0]&4)>0){map[2][0]-=4;map[2][1]-=1;}else{map[2][0]+=4;map[2][1]+=1;}}
void V21(){if((map[2][1]&4)>0){map[2][1]-=4;map[2][2]-=1;}else{map[2][1]+=4;map[2][2]+=1;}}
void V22(){if((map[2][2]&4)>0){map[2][2]-=4;map[2][3]-=1;}else{map[2][2]+=4;map[2][3]+=1;}}
void V30(){if((map[3][0]&4)>0){map[3][0]-=4;map[3][1]-=1;}else{map[3][0]+=4;map[3][1]+=1;}}
void V31(){if((map[3][1]&4)>0){map[3][1]-=4;map[3][2]-=1;}else{map[3][1]+=4;map[3][2]+=1;}}
void V32(){if((map[3][2]&4)>0){map[3][2]-=4;map[3][3]-=1;}else{map[3][2]+=4;map[3][3]+=1;}}
