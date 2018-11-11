import processing.serial.*;
Serial myPort;


public class Point {
    float x = 0;
    float y = 0;
    int sample_count = 1;
    Point() { x = 0; y = 0; }
    Point(float _x, float _y) { x = _x; y = _y; }
    
    float getAngle() {
        return atan2(y , x);
    }
    float getLen() {
        return sqrt(x*x + y*y);   
    }
}



Matrix m2 = new Matrix(3,3);
Point mag_offset = new Point();
Point[]  points = new Point[1024];
Point[]  points_result = new Point[1024];
int points_count = 0;
int points_result_count = 0;

boolean is_recording = false;
 
 
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
float g_angle=0;
float overload=0;



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

    E_CMD_CODE_CALIBRATE_GYRO       = 20,
    E_CMD_CODE_SET_MAGNITUDE_OFFSET = 21,
    E_CMD_CODE_SET_MAGNITUDE_MATRIX = 22,

    E_CMD_CODE_SET_ACC_OFFSET       = 23,
    E_CMD_CODE_SET_ACC_SCALE        = 24,

    E_CMD_CODE_DEBUG_ACTION         = 30,
    E_CMD_CODE_TOGGLE_GYRO          = 31,
    E_CMD_CODE_TOGGLE_MAG           = 32,
    E_CMD_CODE_TOGGLE_ACC           = 33,


    E_CMD_CODE_SAVE                = 40,
    E_CMD_CODE_LOAD                = 41,
    E_CMD_CODE_LOAD_DEFAULT        = 42,
    E_CMD_CODE_TOGGLE_PRINT_MODE   = 43,
    
    the_end = 0
;


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
  byte cmd_code = E_CMD_CODE_NONE;
  
  //if(key == 'd' || key == 'D')  // force
  //  cmd_code = E_CMD_CODE_DEBUG_ACTION;
    if(key == 'z' || key == 'Z') {
        showMessage("Send zero calibration");
        myPort.write(E_CMD_CODE_SET_MAGNITUDE_OFFSET);            
        myPort.write("0.00 0.00 0.0");
        
        myPort.write(E_CMD_CODE_SET_MAGNITUDE_MATRIX);            
        myPort.write("1.00 0.00 0.0 ");
        myPort.write("0.00 1.00 0.0 ");
        myPort.write("0.00 0.00 1.0");                    
    }
  
    if(key == 'r' || key == 'R') {    
        points_count = 0;        
        is_recording = true;
        showMessage("Start REC");
        return;
    }
      
    if(key == 'f' || key == 'F') {
        showMessage("Stop REC");
        is_recording = false;
        ProcessData();
        return;
    }    
    
    if(key == 's' || key == 'S')
        cmd_code = E_CMD_CODE_SAVE;
    if(key == 'l' || key == 'L')
        cmd_code = E_CMD_CODE_LOAD;
    if(key == 'r' || key == 'R')  //recovery
        cmd_code = E_CMD_CODE_LOAD_DEFAULT;  
           
    if(cmd_code != E_CMD_CODE_NONE) {
        println( "Send cmd "+cmd_code);
        myPort.write(cmd_code);
    }
}


void CopyDataToResult() {
    for(int i = 0; i< points_count; i++ ) {        
        points_result[i] = new Point(points[i].x, points[i].y);
    }     
    points_result_count = points_count;    
}

void ProcessData() {  
    if (points_count<=0)
      return;
      
    CopyDataToResult();
    HardIronCalibration();
    SoftIronCalibration();
}

void HardIronCalibration() {       
  Point pmin = new Point(10000,10000);
  Point pmax = new Point(-10000,-10000);
   
  for(int i = 0; i< points_result_count; i++ ) {
    Point p = points_result[i];
    pmin.x = min( pmin.x , p.x);
    pmin.y = min( pmin.y , p.y);
    
    pmax.x = max( pmax.x , p.x);
    pmax.y = max( pmax.y , p.y);    
  }
  
  mag_offset.x = (pmin.x + pmax.x) /2;
  mag_offset.y = (pmin.y + pmax.y) /2;
  applyOffsetToData(-mag_offset.x, -mag_offset.y); 
}

void applyOffsetToData(float x , float y) {
    
    for(int i = 0; i< points_result_count; i++ ) {
        points_result[i].x += x;
        points_result[i].y += y;    
    } 
}

void applyMatrixToData(Matrix mtx) {
    for(int i = 0; i< points_result_count; i++ ) {      
        double[][] ar2 = {{points_result[i].x,points_result[i].y}};
        Matrix m_p = new Matrix(ar2);                
        Matrix m_p2 = m_p.times(mtx);
        points_result[i].x = (float)m_p2.data[0][0];
        points_result[i].y = (float)m_p2.data[0][1];
    }
}

Point FindMinLenPoint() {
    Point pmin = new Point(100000,100000);
    float min_len = 100000;
    
    //find main 
    for(int i = 0; i< points_result_count; i++ ) {
        float len = points_result[i].getLen();
        if(len < min_len) { 
            min_len = len;      
            pmin = points_result[i];
        }                        
    }   
    return pmin;
}

Point FindMaxLenPoint() {
    Point pmax = new Point();
    float max_len = -1;
    
    //find main 
    for(int i = 0; i< points_result_count; i++ ) {
        float len = points_result[i].getLen();            
        if(len > max_len) {
            max_len = len;      
            pmax = points_result[i]; 
        }                         
    }
    return pmax;
}

Matrix CreateScaleMatrix(double scalex, double scaley) {
    double[][] scale_m_data = {
        {scalex, 0     },
        {0     , scaley}        
    };
    return  new Matrix(scale_m_data);
}


void SoftIronCalibration() 
{
    Point pmax = FindMaxLenPoint();
    Point pmin = FindMinLenPoint();

    float min_len = pmin.getLen();
    float max_len = pmax.getLen();
    
    float angle = -pmax.getAngle();    
    //angle = radians(45);
    println( angle * 180 / PI);
    println( "pmax x= "+ pmax.x + " y= "+pmax.y );
    double[][] mat_rot1_data = {
        {cos(angle) , sin(angle)},
        {-sin(angle), cos(angle)}        
    };
    Matrix mat_rot1 = new Matrix(mat_rot1_data );          
    ///////////////////// scale ///////////////////
    float scale = min_len/max_len;
    
    Matrix scale_mat = CreateScaleMatrix(scale, 1);
    
    println( angle * 180 / PI);
    println( "lmin "+ min_len + " lmax "+max_len );
    println( "scale "+ scale );
    
    ///////////////////// rotate back ///////////////
    
    double[][] mat_rot2_data = {
        {cos(-angle) , sin(-angle)},
        {-sin(-angle), cos(-angle)}        
    };
    Matrix mat_rot2 = new Matrix(mat_rot2_data );   
               
    Matrix mat_final = Matrix.identity(2); 
    mat_final = mat_final.times(mat_rot1);
    mat_final = mat_final.times(scale_mat);
    
    // descovery the error - need more scale
    applyMatrixToData(mat_rot1);
    applyMatrixToData(scale_mat);
    pmax = FindMaxLenPoint(); 
    min_len = pmax.getLen();
    scale = 1/scale * min_len/max_len;
    Matrix scale_mat_2 = CreateScaleMatrix(scale, 1);
    
    mat_final = mat_final.times(scale_mat_2);
    mat_final = mat_final.times(mat_rot2);
    applyMatrixToData(scale_mat_2);
    applyMatrixToData(mat_rot2);
    
}

float scale = 450/60;

void drawAxis() {
    float w = width;
    float h = height;
    float len = 5;
    float step = 1;
    stroke(0);
    strokeWeight(1);
    line(0, -h, 0, +h);
    line(-w, 0, w, 0);
    for(float x = -w ; x< +w ; x+=scale*step ) {
      line(x, -len, x, +len);      
    }
    for(float y = -h ; y< +h ; y+=scale*step ) {
      line(-len, y, len, y);      
    }              
}


void drawTextIface() {
    float y = 0;
    float x = 10;
       
    x = width - 200;
    y = 0;
    textSize(32);
    fill(100, 100, 100);
    text("magx " + magx, x, y+=30); 
    //fill(0, 100, 0);
    text("magy " + magy, x, y+=30);
    //fill(0, 0, 100);
    text("magz " + magz, x, y+=30);
    
    
    textSize(32);
    fill(100, 100, 100);
    text("mag_offset_x " + mag_offset.x, x, y+=30); 
    //fill(0, 100, 0);
    text("mag_offset_y " + mag_offset.y, x, y+=30);
    //fill(0, 0, 100);
    //text("mag_offset_y  " + magz, x, y+=30);

    
    
    textSize(24);
    x = width / 2;
    y = height /2 - 100;
    float alpa = last_message_timer / last_message_time_max * 255; 
    fill(128, 0, 0, alpa);
    textAlign(CENTER);
    text(last_message, x, y+=14 );
       
}


void drawData() 
{   
  stroke(255,128,128);
  strokeWeight(4);  // Thicker
  for(int i=1; i<points_count; i++) {
    Point p1 = points[i-1];
    Point p2 = points[i];
    line(p1.x * scale, p1.y * scale, p2.x * scale, p2.y * scale);   
  }
  
  stroke(128,128,255);
  strokeWeight(4);  // Thicker
  for(int i=1; i<points_result_count; i++) {
    Point p1 = points_result[i-1];
    Point p2 = points_result[i];
    line(p1.x * scale, p1.y * scale, p2.x * scale, p2.y * scale);   
  }
  
  
}


void storeData() {
  if(!is_recording)
    return;
    
  Point p = new Point();
  p.sample_count = 1;
  p.x = magx;
  p.y = magy;
  
  float angle_delta = PI / 180 * 2;
  float angle = p.getAngle();  
  Point p2 = null;
  //for(int i =0; i< points_count;i++) {
  //   float prev_angle = points[i].getAngle();
  //   if(abs(prev_angle - angle) < angle_delta ) {
  //      p2 = points[i];
  //      break;
  //   }             
  //}        
  
  if(p2 != null) {       
    int c = p2.sample_count + p.sample_count;
    p2.x = (p2.x * p2.sample_count + p.x * p.sample_count ) / c;
    p2.y = (p2.y * p2.sample_count + p.y * p.sample_count ) / c;
    points[points_count-1] = p2;       
  } else {  
    points[points_count] = p;
    points_count++;
    if(points_count >= points.length) {
      showMessage("new lap");      
      for(int i = 0; i < points_count-512; i++) {
          points[i] = points[i+512];          
      }
      points_count-=512;
      //is_recording = false;
    }
      
  }
}

void draw()
{
    serialEvent();  // read and parse incoming serial message
    background(255); // set background to white
    lights();
   
    if(last_message_timer > 0) 
      last_message_timer -= 0.1;
    
                                                                                                                              
    //line(mouseX-66, mouseY, mouseX+66, mouseY);
    //line(mouseX, mouseY-66, mouseX, mouseY+66);
        
    drawTextIface();
    
    
    
    
    pushMatrix();
    translate(width*0.5,height*0.5);
    drawAxis();
    drawData();
    
    popMatrix();
    
}


void showMessage(String message) {
  last_message = message;
  last_message_timer = last_message_time_max;
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
      
      if(yaw <0)
        yaw += 360;
        
      storeData();
      g= sqrt(accx*accx + accy*accy + accz*accz);
      g_angle = -degrees(atan2( accy, accz));
      overload = g / 9.8;
      continue;
    } 
    
    if(message.length() > 0  ) {
      showMessage(message);
    }
    println(message);      
  } while (true);
}
