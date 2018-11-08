import processing.serial.*;
Serial myPort;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;


float accx = 0.0;
float accy = 0.0;
float accz = 0.0;

float gerrx = 0.0;
float gerry = 0.0;
float gerrz = 0.0;

float temp = 0.0;

String last_message = "Hello";

int
E_CMD_CODE_NONE = 0,
E_CMD_CODE_RESET_PITCH_ROLL      = 1,  // сбросить крен тангаж
E_CMD_CODE_SET_YAW_BY_MAG        = 2,  // установить углы по магнетометру
E_CMD_CODE_SET_PITCH_ROLL_BY_ACC = 3,  // установить углы по акселерометру
E_CMD_CODE_BOOST_FILTER          = 4,  // установить углы по акселерометру

E_CMD_CODE_SET_GRAVITY_VECTOR  = 10,  // текущее направление силы тяжести принять за 0 (roll pitch)
E_CMD_CODE_SET_YAW_NORTH       = 11,  // текущее направление на север принять за 0 (yaw)

E_CMD_CODE_CALIBRATE_GYRO      = 20,  //

E_CMD_CODE_DEBUG_ACTION        = 30,  
E_CMD_CODE_TOGGLE_GYRO         = 31,  
E_CMD_CODE_TOGGLE_MAG          = 32,  
E_CMD_CODE_TOGGLE_ACC          = 33,  
the_end = 0
;


int[] cmd_Array = {
  //E_CMD_CODE_NONE,  
  E_CMD_CODE_TOGGLE_MAG,
  E_CMD_CODE_RESET_PITCH_ROLL,
  E_CMD_CODE_SET_YAW_BY_MAG,  // установить углы по магнетометру
  E_CMD_CODE_SET_PITCH_ROLL_BY_ACC,  // установить углы по акселерометру
  E_CMD_CODE_BOOST_FILTER,  // установить углы по акселерометру
  
  E_CMD_CODE_SET_GRAVITY_VECTOR,  // текущее направление силы тяжести принять за 0 (roll pitch)
  E_CMD_CODE_SET_YAW_NORTH,  // текущее направление на север принять за 0 (yaw)
  
  E_CMD_CODE_CALIBRATE_GYRO,  //
  
  E_CMD_CODE_DEBUG_ACTION,
  E_CMD_CODE_TOGGLE_GYRO,   
  E_CMD_CODE_TOGGLE_ACC
};

void setup()
{
  size(900, 900, P3D);

  // if you have only ONE serial port active
  //myPort = new Serial(this, Serial.list()[1], 115200); // if you have only ONE serial port active

  // if you know the serial port name
  //myPort = new Serial(this, "COM5:", 9600);        // Windows "COM#:"
  //myPort = new Serial(this, "\\\\.\\COM41", 9600); // Windows, COM10 or higher
  myPort = new Serial(this, "/dev/ttyUSB0", 115200);   // Linux "/dev/ttyACM#"
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 9600);  // Mac "/dev/cu.usbmodem######"

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

void keyPressed() {
  //println("Key = " + int(key));
  int keyIndex = -1;
  if (key >= '0' && key <= '9') 
    keyIndex = key - '0';  
  if(key == 45)
    keyIndex = 10;
  if(key == 61)
    keyIndex = 11;
     
  if(keyIndex < 0 || keyIndex >= cmd_Array.length)     
    return;
   
  byte cmd_code = byte(cmd_Array[keyIndex]);
  println( "Send cmd "+cmd_code);
  
  myPort.write(cmd_code);
}

void drawObject(float roll, float pitch, float yaw, float x , float y) {
  
  pushMatrix(); // begin object
  translate(x, y);
  
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(-pitch));
  float s2 = sin(radians(-pitch));
  float c3 = cos(radians(-yaw+90));
  float s3 = sin(radians(-yaw+90));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  drawPropShield();  
  popMatrix(); // end of object   
}

void drawObject2(float roll, float pitch, float yaw, float x , float y) {
  
  pushMatrix(); // begin object
  translate(x, y);
  
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(-pitch));
  float s2 = sin(radians(-pitch));
  float c3 = cos(radians(-yaw+90));
  float s3 = sin(radians(-yaw+90));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  drawHorisont();  
  popMatrix(); // end of object   
}


void drawTextIface() {
    float y = 0;
    float x = 10;
    textAlign(LEFT);
    textSize(32);
    fill(100, 0, 0);
    text("roll    " + roll, x, y+=30); 
    fill(0, 100, 0);
    text("pitch " + pitch, x, y+=30);
    fill(0, 0, 100);
    text("yaw   " + yaw, x, y+=30);
    
    textSize(24);
    fill(100, 0, 0);
    text("roll err  " + nf(gerrx,1,5), x, y+=30); 
    fill(0, 100, 0);
    text("pitch err " + nf(gerry,1,5), x, y+=24);
    fill(0, 0, 100);
    text("yaw err   " + nf(gerrz,1,5), x, y+=24);
    
    
    x = width - 200;
    y = 0;
    textSize(32);
    fill(100, 100, 100);
    text("accx " + accx, x, y+=30); 
    //fill(0, 100, 0);
    text("accy " + accy, x, y+=30);
    //fill(0, 0, 100);
    text("accz " + accz, x, y+=30);
    float g= sqrt(accx*accx + accy*accy + accz*accz);
    float g_angle = degrees(atan2( accy, accz));
    float overload = g  / 9.8;
       
    fill(100, 100, 0);
    text("g " + nf(g,1,2), x, y+=30);
    text("a " + int(g_angle), x, y+=30);
    text("overload " + nf(overload, 1,1), x, y+=30);
    
    x = width/2 - 100;
    y = 0;
    fill(100, 0, 0);
    text("temp " + temp, x, y+=30);
    
   
    textSize(14);
    x = 10;
    y = height - 300;
    fill(0, 0, 0, 128);
    text("1 - E_CMD_CODE_RESET_PITCH_ROLL ", x, y+=14 );
    text("2 - E_CMD_CODE_SET_YAW_BY_MAG ", x, y+=14 );
    text("3 - E_CMD_CODE_SET_PITCH_ROLL_BY_ACC ", x, y+=14 );
    text("4 - E_CMD_CODE_BOOST_FILTER ", x, y+=14 );
    text("5 - E_CMD_CODE_SET_GRAVITY_VECTOR ", x, y+=14 );
    text("6 - E_CMD_CODE_SET_YAW_NORTH ", x, y+=14 );
    text("7 - E_CMD_CODE_CALIBRATE_GYRO ", x, y+=14 );
    text("8 - E_CMD_CODE_DEBUG_ACTION ", x, y+=14 );
    text("9 - E_CMD_CODE_TOGGLE_GYRO ", x, y+=14 );
    text("0 - E_CMD_CODE_TOGGLE_MAG ", x, y+=14 );    
    text("- - E_CMD_CODE_TOGGLE_ACC ", x, y+=14 );             
    
    
    
    textSize(24);
    x = width / 2;
    y = height - 24;
    fill(128, 0, 0);
    textAlign(CENTER);
    text(last_message, x, y+=14 );
    
    
}

void draw()
{
    serialEvent();  // read and parse incoming serial message
    background(255); // set background to white
    lights();
    
    drawObject2(roll, 0    , 0   , width*0.5 , height*0.5);
    drawObject (roll, pitch, yaw , width*0.5 , height*0.5);      
    drawObject (roll, 0    , 0   , width*0.5 , height*0.75);  
    drawObject (0   , pitch, 0   , width*0.75, height*0.5);
    drawObject (0   , pitch, 0   , width*0.25, height*0.5);
    drawObject (0   , 0    , yaw , width*0.5 , height*0.15);         
            
    //line(mouseX-66, mouseY, mouseX+66, mouseY);
    //line(mouseX, mouseY-66, mouseX, mouseY+66);
    
    drawTextIface();
    
}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message == null)
      break;
    
    message = trim(message);
    String[] list = split(message, " ");
    if (list.length >= 10 && list[0].equals("Orient:")) {               
      roll   = float(list[1]); // convert to float roll
      pitch  = float(list[2]); // convert to float pitch
      yaw    = float(list[3]) ; // convert to float yaw
      
      accx = float(list[5]) ; // convert to float yaw
      accy = float(list[6]) ; // convert to float yaw
      accz = float(list[7]) ; // convert to float yaw
      
      temp = float(list[9]);
      
      gerrx = float(list[11]) / 1000;
      gerry = float(list[12]) / 1000;
      gerrz = float(list[13  ]) / 1000;
      
      continue;
    } 
    
    if(message.length() > 1)
      last_message = message; 
    println(message);      
  } while (true);
}

void drawArduino()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(300, 10, 200); // draw Arduino board base shape

  stroke(0); // set outline colour to black
  fill(80); // set fill colour to dark grey

  translate(60, -10, 90); // set position to edge of Arduino box
  box(170, 20, 10); // draw pin header as box

  translate(-20, 0, -180); // set position to other edge of Arduino box
  box(210, 20, 10); // draw other pin header as box
}

void drawHorisont()
{
    /* function contains shape(s) that are rotated with the IMU */
    //stroke(0, 90, 90); // set outline colour to darker teal
    
    //fill(128, 128, 250); // sky    
    //translate(-100, -300, 0); // set position to edge of Arduino box
    //box(1, 600, 1200); // draw Arduino board base shape       
    
    //stroke(0, 90, 90); // set outline colour to darker teal
    //fill(0, 128, 16); // ground
    //translate(0, 600, 0); // set position to edge of Arduino box
    //box(1, 600, 1200); // draw Arduino board base shape
        
    stroke(0, 90, 90); // set outline colour to darker teal
    fill(0, 128, 16); // ground
    translate(-100, -30, 0); // set position to edge of Arduino box
    box(1, 60, 1200); // draw Arduino board base shape
}

void drawPropShield()
{
  // 3D art by Benjamin Rheinland
  stroke(0); // black outline
  fill(0, 128, 0); // fill color PCB green
  box(190, 6, 70); // PCB base shape

  fill(255, 215, 0); // gold color
  noStroke();

  //draw 14 contacts on Y- side
  translate(65, 0, 30);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(-10, 0, 0); // set new position
  }

  //draw 14 contacts on Y+ side
  translate(10, 0, -60);
  for (int i=0; i<14; i++) {
    sphere(4.5); // draw gold contacts
    translate(10, 0, 0); // set position
  }

  //draw 5 contacts on X+ side (DAC, 3v3, gnd)
  translate(-10,0,10);
  for (int i=0; i<5; i++) {
    sphere(4.5);
    translate(0,0,10);
  }

  //draw 4 contacts on X+ side (G C D 5)
  translate(25,0,-15);
  for (int i=0; i<4; i++) {
    sphere(4.5);
    translate(0,0,-10);
  }

  //draw 4 contacts on X- side (5V - + GND)
  translate(-180,0,10);
  for (int i=0; i<4; i++) {
    sphere(4.5);
    translate(0,0,10);
  }

  //draw audio amp IC
  stroke(128);
  fill(24);    //Epoxy color
  translate(30,-6,-25);
  box(13,6,13);

  //draw pressure sensor IC
  stroke(64);
  translate(32,0,0);
  fill(192);
  box(10,6,18);

  //draw gyroscope IC
  stroke(128);
  translate(27,0,0);
  fill(24);
  box(16,6,16);

  //draw flash memory IC
  translate(40,0,-15);
  box(20,6,20);

  //draw accelerometer/magnetometer IC
  translate(-5,0,25);
  box(12,6,12);

  //draw 5V level shifter ICs
  translate(42.5,2,0);
  box(6,4,8);
  translate(0,0,-20);
  box(6,4,8);
}
