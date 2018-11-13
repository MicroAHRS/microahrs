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


float magx = 0.0;
float magy = 0.0;
float magz = 0.0;

float temp = 0.0;

float beta = 0;
float zeta = 0;

float last_message_time_max = 30.0;
float last_message_timer = 0.0;
String last_message = "Hello";

float g=0;
float g_anlge_roll=0;
float g_angle_pitch=0;
float mag_angle_yaw=0;
float overload=0;

float fps= 0;

byte
  E_CMD_CODE_NONE = 0,
    E_CMD_CODE_RESET_PITCH_ROLL      = 1,  // сбросить крен тангаж
    E_CMD_CODE_SET_YAW_BY_MAG        = 2,  // установить углы по магнетометру
    E_CMD_CODE_SET_PITCH_ROLL_BY_ACC = 3,  // установить углы по акселерометру
    E_CMD_CODE_BOOST_FILTER          = 4,  // установить углы по акселерометру

    E_CMD_CODE_CHANGE_BETA           = 5,
    E_CMD_CODE_CHANGE_ZETA           = 6,

    E_CMD_CODE_SET_GRAVITY_VECTOR  = 10,  // текущее направление силы тяжести принять за 0 (roll pitch)
    E_CMD_CODE_SET_YAW_NORTH       = 11,  // текущее направление на север принять за 0 (yaw)
    E_CMD_CODE_DEFAULT_ORIENTATION = 12,  // сбросить модификатор ориентации

    E_CMD_CODE_CALIBRATE_GYRO       = 20,
    E_CMD_CODE_SET_MAGNITUDE_OFFSET = 21,
    E_CMD_CODE_SET_MAGNITUDE_MATRIX = 22,

    E_CMD_CODE_SET_ACC_OFFSET       = 23,
    E_CMD_CODE_SET_ACC_SCALE        = 24,

    E_CMD_CODE_DEBUG_ACTION         = 30,
    E_CMD_CODE_TOGGLE_GYRO          = 31,
    E_CMD_CODE_CALIBRATION_STOP     = 32,  // code = space - useful when send calibration
    E_CMD_CODE_TOGGLE_MAG           = 33,
    E_CMD_CODE_TOGGLE_ACC           = 34,


    E_CMD_CODE_SAVE                = 40,
    E_CMD_CODE_LOAD                = 41,
    E_CMD_CODE_LOAD_DEFAULT        = 42,
    E_CMD_CODE_TOGGLE_PRINT_MODE   = 43,
    the_end = 0
;


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
    
    myPort.write(E_CMD_CODE_BOOST_FILTER);
    textSize(16); // set text size
    textMode(SHAPE); // set text mode to shape
}


void drawTextCommand() {
    float x = width/2 - 150;
    float y = 60;
    float h= 14;
    textSize(h = 14);
    fill(0, 0, 0);
    
    text("q - RESET_PITCH_ROLL ", x, y+=h );
    text("c - CALIBRATE_GYRO ", x, y+=h );         
    text("b - CHANGE_BETA ", x, y+=h );
    text("z - CHANGE_ZETA ", x, y+=h );
    text("f - BOOST_FILTER ", x, y+=h );
    y+=h/2;
    text("g - TOGGLE_GYRO ", x, y+=h );
    text("m - TOGGLE_MAG ", x, y+=h );    
    text("a - TOGGLE_ACC ", x, y+=h );
    text("d - DEBUG_ACTION ", x, y+=h );
    y+=h/2;
    text("s - SAVE ", x, y+=h );
    text("l - LOAD ", x, y+=h );
    text("r - LOAD_DEFAULT ", x, y+=h );
    y+=h/2;
    text("p - SET_PITCH_AND_ROLL", x, y+=h );    
    text("y - SET_YAW ", x, y+=h );
    text("i - DEFAULT_ORIENTATION ", x, y+=h );
    
    
    //text("r - E_CMD_CODE_LOAD_DEFAULT ", x, y+=h );
}

void keyPressed() {
    println("Key = " + int(key));
  int keyIndex = -1;  
    
  byte cmd_code = E_CMD_CODE_NONE;
   
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
  if(key == 'r' || key == 'R')  //recovery
    cmd_code = E_CMD_CODE_LOAD_DEFAULT;  
  if(key == 'y' || key == 'Y')  //yaw
    cmd_code = E_CMD_CODE_SET_YAW_NORTH;
  if(key == 'p' || key == 'P')  // pitch
    cmd_code = E_CMD_CODE_SET_GRAVITY_VECTOR;
  if(key == 'i' || key == 'I')  // pitch
    cmd_code = E_CMD_CODE_DEFAULT_ORIENTATION;
   
  //if(key == 'k' || key == 'K') {
  //  cmd_code = E_CMD_CODE_SET_ACC_OFFSET;
  //  myPort.write(cmd_code);    
  //  myPort.write("0.00 0.00 0.00");
  //  //myPort.write("1.00 0.00 0.00 0.00 1.00 0.00 0.00 0.00 1.00");
  //  return;
  //}
  
  //if(key == 'l' || key == 'L') {
  //  cmd_code = E_CMD_CODE_SET_ACC_OFFSET;
  //  myPort.write(cmd_code);
  //  accz = 0.6124513;
  //  myPort.write(accx + " " + accy + " "+ accz); 
  //  return;
  //}
      
  
  if(cmd_code != E_CMD_CODE_NONE) {
    println( "Send cmd "+cmd_code);
    myPort.write(cmd_code);
  }
}



void drawObject2(float roll, float pitch, float yaw, float x , float y) {
  
    pushMatrix(); // begin object
    translate(x, y, -100);
    
    //float c1 = cos(radians(roll));
    //float s1 = sin(radians(roll));
    //float c2 = cos(radians(pitch * 0));
    //float s2 = sin(radians(pitch * 0));
    //float c3 = cos(radians(90));
    //float s3 = sin(radians(90));
    //applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
    //             -s2, c1*c2, c2*s1, 0,
    //             c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
    //             0, 0, 0, 1);
    
    rotate(radians(-roll));
                 
    float pixels_per_degree = 10;
    translate(0,pitch * pixels_per_degree);
    
    
    pushMatrix(); // begin object
    stroke(0);
    fill(0, 0, 0); // ground       
    box(2400 * 2, 1,2);
        
    fill(128, 255, 128); // ground
    translate(0, 600 * 2); // set position to edge of Arduino box
    box(2400 * 2, 1200 * 2,1);
    popMatrix(); // end of object
       
    translate(0, 0, 10); 
    noStroke(); 
    fill(0, 0, 0); 
    float h = 100;
    float angle_step = 10;    
    for(int ya = -360; ya <= +360 + 360 ; ya += angle_step) {
      float lx = (-yaw + ya) * pixels_per_degree;      
      h=(ya % 45 == 0) ? 100 : 50;      
      float w=(ya % 45 == 0) ? 2 : 1;
      pushMatrix();
      translate(lx, 0, 10); 
      //line(0, -h, 0, +h);
      box(w, h*2, 1);
      popMatrix(); 
    }
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
    //text("magx " + magx, x, y+=30); 
    ////fill(0, 100, 0);
    //text("magy " + magy, x, y+=30);
    ////fill(0, 0, 100);
    //text("magz " + magz, x, y+=30);
    
    //fill(100, 100, 100);
    //text("accx " + accx, x, y+=30); 
    ////fill(0, 100, 0);
    //text("accy " + accy, x, y+=30);
    ////fill(0, 0, 100);
    //text("accz " + accz, x, y+=30);
    
       
    fill(100, 100, 0);
    text("g " + nf(g,1,2), x, y+=30);
    //text("a " +  nf(g_anlge_roll,1,2), x, y+=30);
    text("overload " + nf(overload, 1,1), x, y+=30);
    
    x = width/2 - 100;
    y = 0;
    fill(100, 0, 0);
    text("temp " + temp, x, y+=30);
    text("fps " + fps, x, y+=30);
    
               
    drawTextCommand();
    
    
    
    textSize(24);
    x = width / 2;
    y = height /2 - 100;
    float alpa = last_message_timer / last_message_time_max * 255; 
    fill(128, 0, 0, alpa);
    textAlign(CENTER);
    text(last_message, x, y+=14 );
       
}

void drawCursor(float x, float y,float angle, PImage img, boolean flip, float anlge_aim, boolean rev) {
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
  
  pushMatrix(); // begin aim indicator  
  rotate(radians(-anlge_aim));
  translate(0, (rev?-1:1)*250 / 2);
  fill(128, 255, 128);
  float d2 = 10;
  ellipseMode(CENTER);
  ellipse(0, 0, d2, d2);   
  popMatrix(); // end of object 
  
  popMatrix(); // end of object
  
  
 
   
  textAlign(CENTER);
  textSize(32);
  fill(0, 0, 0);
  text(nf(angle,3,2), x, y+= (d/2 + 32));         
}

void drawGravityIndicator(float x, float y, float angle,float d) 
{
  pushMatrix(); // begin object
  translate(x, y);
  rotate(radians(angle));
  translate(0, y * 0.95f);     
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
    
    drawObject2(roll, pitch    , yaw   , width*0.5 , height*0.5);
    stroke(0);
    
    drawCursor(width*0.5 , height*0.75,roll, g_cursor, false, g_anlge_roll, false);
    drawCursor(width*0.80 , height*0.75,pitch, g_pitch_cursor, roll > 90 || roll < - 90, g_angle_pitch, false);  
    drawCursor(width*0.20 , height*0.75,yaw, g_yaw_cursor, false,mag_angle_yaw,true);
    
    image(g_cross,width*0.5 -g_cross.width /2 ,height*0.5 -g_cross.height * 0);
    fill(200, 200,200);
    drawGravityIndicator(width*0.5 , height*0.5, 0, 40);
    fill(100, 200,200);
    drawGravityIndicator(width*0.5 , height*0.5, -g_anlge_roll, 20);
                 
                                                                                                                              
    //line(mouseX-66, mouseY, mouseX+66, mouseY);
    //line(mouseX, mouseY-66, mouseX, mouseY+66);
    
    drawTextIface();
    
}

Matrix getRollMatrix(float a) 
{    
    double[][] d = { 
        {1,0,0},
        {0,cos(a),-sin(a)},
        {0,sin(a), cos(a)}       
    };
    return new Matrix(d);
}

Matrix getPitchMatrix(float a) 
{    
    double[][] d = { 
        {cos(a),0,sin(a)},
        {0,1,0},
        {-sin(a),0, cos(a)}       
    };
    return new Matrix(d);
}


void RotateMag() {
    double[][] d = {{magx,magy,magz}};
    Matrix mag_m = new Matrix(d);
    
    //Matrix m = getPitchMatrix(radians(pitch)).times(getRollMatrix(radians(roll)));
    Matrix m = getRollMatrix(radians(-roll)).times(getPitchMatrix(radians(pitch)));
    Matrix r = mag_m.times(m);
    magx = (float)r.data[0][0];
    magy = (float)r.data[0][1];
    magz = (float)r.data[0][2];         
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
      roll   = float(list[1]); // convert to float roll
      pitch  = -float(list[2]); // convert to float pitch
      yaw    = -float(list[3]) ; // convert to float yaw
      
      accx = float(list[5]) ; 
      accy = float(list[6]) ; 
      accz = float(list[7]) ; 
      
      temp = float(list[9]);
      
      gerrx = float(list[11]) / 1000;
      gerry = float(list[12]) / 1000;
      gerrz = float(list[13]) / 1000;
      
      beta = float(list[14]) / 1000;
      zeta = float(list[15]) / 1000;
      
      if (list.length > 19) {
        magx = float(list[17]) ; 
        magy = float(list[18]) ; 
        magz = float(list[19]) ;
      }
      
      if(list.length > 21) 
        fps = float(list[21]) ;     
      
      if(yaw <0)
        yaw += 360;
        
      g= sqrt(accx*accx + accy*accy + accz*accz);
      g_anlge_roll = degrees(atan2( accy, accz));
      g_angle_pitch = degrees(atan2( accx, accz));
      
      //rotate mag by roll
      //rotate mag by pitch
      
      RotateMag();
            
      mag_angle_yaw = degrees(atan2( magy, magx));
      
      
      overload = g / 9.8;
      continue;
    } 
    
    if(message.length() > 0  ) {
      last_message = message;
      last_message_timer = last_message_time_max;
    }
    println(message);      
  } while (true);
}
