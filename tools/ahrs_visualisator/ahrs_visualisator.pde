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

float beta = 0;
float zeta = 0;

float last_message_time_max = 30.0;
float last_message_timer = 0.0;
String last_message = "Hello";

float g=0;
float g_angle=0;
float overload=0;

byte
    E_CMD_CODE_NONE = 0,
    E_CMD_CODE_RESET_PITCH_ROLL      = 1,  // сбросить крен тангаж
    E_CMD_CODE_SET_YAW_BY_MAG        = 2,  //  bad
    E_CMD_CODE_SET_PITCH_ROLL_BY_ACC = 3,  // bad
    E_CMD_CODE_BOOST_FILTER          = 4,   
    E_CMD_CODE_CHANGE_BETA           = 5,  
    E_CMD_CODE_CHANGE_ZETA           = 6,
    
    E_CMD_CODE_SET_GRAVITY_VECTOR  = 10,  // текущее направление силы тяжести принять за 0 (roll pitch)
    E_CMD_CODE_SET_YAW_NORTH       = 11,  // текущее направление на север принять за 0 (yaw)
    
    E_CMD_CODE_CALIBRATE_GYRO      = 20,  //
    
    E_CMD_CODE_DEBUG_ACTION        = 30,  
    E_CMD_CODE_TOGGLE_GYRO         = 31,  
    E_CMD_CODE_TOGGLE_MAG          = 32,  
    E_CMD_CODE_TOGGLE_ACC          = 33,  
    
    E_CMD_CODE_SAVE                = 40,
    E_CMD_CODE_LOAD                = 41,
    
    the_end = 0
;


int[] cmd_Array = {
  //E_CMD_CODE_NONE,  
  E_CMD_CODE_TOGGLE_MAG,
  E_CMD_CODE_RESET_PITCH_ROLL,
  E_CMD_CODE_CHANGE_BETA,  
  E_CMD_CODE_CHANGE_ZETA,  
  E_CMD_CODE_BOOST_FILTER,  // установить углы по акселерометру
  
  E_CMD_CODE_SET_GRAVITY_VECTOR,  // текущее направление силы тяжести принять за 0 (roll pitch)
  E_CMD_CODE_SET_YAW_NORTH,  // текущее направление на север принять за 0 (yaw)
  
  E_CMD_CODE_CALIBRATE_GYRO,  //
  
  E_CMD_CODE_DEBUG_ACTION,
  E_CMD_CODE_TOGGLE_GYRO,   
  E_CMD_CODE_TOGGLE_ACC
};

PImage g_cross;
PImage g_cursor;
PImage g_pitch_cursor;
PImage g_yaw_cursor;

void setup()
{
  size(900, 900, P3D);


  g_cross = loadImage("horizon3.png");
  //g_cursor = loadImage("cursor.png");  
  g_cursor = loadImage("FrontPlaneIcon2.png");
  g_pitch_cursor = loadImage("pitch_indicator.png");
  g_yaw_cursor = loadImage("yaw_indicator.png");
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


void drawTextCommand() {
    float x = width/2 - 150;
    float y = 60;
    float h= 14;
    textSize(h = 18);
    fill(0, 0, 0);
    
    text("q - E_CMD_CODE_RESET_PITCH_ROLL ", x, y+=h );
    text("c - E_CMD_CODE_CALIBRATE_GYRO ", x, y+=h );         
    text("b - E_CMD_CODE_CHANGE_BETA ", x, y+=h );
    text("z - E_CMD_CODE_CHANGE_ZETA ", x, y+=h );
    text("f - E_CMD_CODE_BOOST_FILTER ", x, y+=h );       
    text("g - E_CMD_CODE_TOGGLE_GYRO ", x, y+=h );
    text("m - E_CMD_CODE_TOGGLE_MAG ", x, y+=h );    
    text("a - E_CMD_CODE_TOGGLE_ACC ", x, y+=h );
    text("d - E_CMD_CODE_DEBUG_ACTION ", x, y+=h );
}

void keyPressed() {
    println("Key = " + int(key));
  int keyIndex = -1;
  if (key >= '0' && key <= '9') 
    keyIndex = key - '0';  
  if(key == 45)
    keyIndex = 10;
  if(key == 61)
    keyIndex = 11;
    
  byte cmd_code = E_CMD_CODE_NONE;
  if(keyIndex >= 0 && keyIndex < cmd_Array.length) {          
    cmd_code = byte(cmd_Array[keyIndex]);    
  }
  
  if(key == 'd' || key == 'D')  // force
    cmd_code = E_CMD_CODE_DEBUG_ACTION;
  if(key == 'c' || key == 'C')  // force
    cmd_code = E_CMD_CODE_CALIBRATE_GYRO;
  if(key == 'q' || key == 'Q')  // force
    cmd_code = E_CMD_CODE_RESET_PITCH_ROLL;
  if(key == 'f' || key == 'F')  // force
    cmd_code = E_CMD_CODE_BOOST_FILTER;
  if(key == 'z' || key == 'Z')
    cmd_code = E_CMD_CODE_CHANGE_ZETA;
  if(key == 'b' || key == 'B')
    cmd_code = E_CMD_CODE_CHANGE_BETA;
  if(key == 'm' || key == 'M')
    cmd_code = E_CMD_CODE_TOGGLE_MAG;
  if(key == 'a' || key == 'A')
    cmd_code = E_CMD_CODE_TOGGLE_ACC;
  if(key == 'g' || key == 'G')
    cmd_code = E_CMD_CODE_TOGGLE_GYRO;
  if(key == 's' || key == 'S')
    cmd_code = E_CMD_CODE_SAVE;
  if(key == 'l' || key == 'L')
    cmd_code = E_CMD_CODE_LOAD;  
      
  
  if(cmd_code != E_CMD_CODE_NONE) {
    println( "Send cmd "+cmd_code);
    myPort.write(cmd_code);
  }
}



void drawObject2(float roll, float pitch, float yaw, float x , float y) {
  
  pushMatrix(); // begin object
  translate(x, y);
  
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch * 0));
  float s2 = sin(radians(pitch * 0));
  float c3 = cos(radians(90));
  float s3 = sin(radians(90));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);
  translate(0,pitch * 10);

  
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
    fill(100, 0, 100);
    text("roll err  " + nf(gerrx,1,5), x, y+=30);     
    text("pitch err " + nf(gerry,1,5), x, y+=24);    
    text("yaw err   " + nf(gerrz,1,5), x, y+=24);
    
    y+=24;
    fill(100, 100, 0);
    text("beta " + nf(beta,1,5), x, y+=24);     
    text("zeta " + nf(zeta,1,5), x, y+=24);        
    
    
    x = width - 200;
    y = 0;
    textSize(32);
    fill(100, 100, 100);
    text("accx " + accx, x, y+=30); 
    //fill(0, 100, 0);
    text("accy " + accy, x, y+=30);
    //fill(0, 0, 100);
    text("accz " + accz, x, y+=30);
    
       
    fill(100, 100, 0);
    text("g " + nf(g,1,2), x, y+=30);
    text("a " +  nf(g_angle,1,2), x, y+=30);
    text("overload " + nf(overload, 1,1), x, y+=30);
    
    x = width/2 - 100;
    y = 0;
    fill(100, 0, 0);
    text("temp " + temp, x, y+=30);
    
               
    drawTextCommand();
    
    
    
    textSize(24);
    x = width / 2;
    y = height /2 - 100;
    float alpa = last_message_timer / last_message_time_max * 255; 
    fill(128, 0, 0, alpa);
    textAlign(CENTER);
    text(last_message, x, y+=14 );
       
}

void drawCursor(float x, float y,float angle, PImage img, boolean flip) {
  pushMatrix(); // begin object
  translate(x, y);
  rotate(radians(angle));  
  fill(200);
  float w2 = img.width / 2;
  float h2 = img.height / 2;
  float d = 260;
  ellipseMode(CENTER);
  ellipse(0, 0, d, d);
  if(flip)
     scale(1,-1);
  image(img, -w2 , -h2);   
  popMatrix(); // end of object
   
  textAlign(CENTER);
  textSize(32);
  fill(0, 0, 0);
  text(nf(angle,3,2), x, y+= (d/2 + 32));         
}

void drawGravityIndicator(float x, float y, float angle) 
{
  pushMatrix(); // begin object
  translate(x, y);
  rotate(radians(angle));
  translate(0, y * 0.95f);
  fill(100, 200,200);  
  float d = 20;
  ellipseMode(CENTER);
  ellipse(0, 0, d, d);   
  popMatrix(); // end of object            
}

void draw()
{
    serialEvent();  // read and parse incoming serial message
    background(255); // set background to white
    lights();
   
    if(last_message_timer > 0) 
      last_message_timer -= 0.1;
    
    drawObject2(roll, pitch    , 0   , width*0.5 , height*0.5);
    
    drawCursor(width*0.5 , height*0.75,roll, g_cursor, false);
    drawCursor(width*0.80 , height*0.75,pitch, g_pitch_cursor, roll > 90 || roll < - 90);  
    drawCursor(width*0.20 , height*0.75,yaw, g_yaw_cursor, false);
    
    image(g_cross,width*0.5 -g_cross.width /2 ,height*0.5 -g_cross.height * 0);
    drawGravityIndicator(width*0.5 , height*0.5, -g_angle);
                 
                     
                                                                                              
          
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
    if (list.length > 15 && list[0].equals("Orient:")) {               
      roll   = -float(list[1]); // convert to float roll
      pitch  = float(list[2]); // convert to float pitch
      yaw    = -float(list[3]) ; // convert to float yaw
      
      accx = float(list[5]) ; // convert to float yaw
      accy = float(list[6]) ; // convert to float yaw
      accz = float(list[7]) ; // convert to float yaw
      
      temp = float(list[9]);
      
      gerrx = float(list[11]) / 1000;
      gerry = float(list[12]) / 1000;
      gerrz = float(list[13]) / 1000;
      
      beta = float(list[14]) / 1000;
      zeta = float(list[15]) / 1000;
      
      if(yaw <0)
        yaw += 360;
        
      g= sqrt(accx*accx + accy*accy + accz*accz);
      g_angle = -degrees(atan2( accy, accz));
      overload = g / 9.8;
      continue;
    } 
    
    if(message.length() > 1) {
      last_message = message;
      last_message_timer = last_message_time_max;
    }
    println(message);      
  } while (true);
}

void drawHorisont()
{       
    stroke(0, 90, 90); // set outline colour to darker teal
    fill(128, 255, 128); // ground
    translate(-100, 600 * 2, 0); // set position to edge of Arduino box
    box(1, 1200 * 2, 2400 * 2); 
}
