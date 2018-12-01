import processing.serial.*;


float fps;
Matrix mag_matrix = Matrix.identity(2);
Point  mag_offset = new Point();
Point[]  points = new Point[1024];
Point[]  points_result = new Point[1024];
int points_count = 0;
int points_result_count = 0;

boolean is_recording = false;

float FLOAT_FACKTOR = 1000;

SensorData  sensor_data = new SensorData();
Serial sensor_port;

float last_message_time_max = 30.0;
float last_message_timer = 0.0;
String last_message = "Hello";

float float_factor = 1000000;

float scale = 450/60;


void setup()
{    
    size(900, 900, P3D); 
     setupSerial();
    textSize(16); // set text size
    textMode(SHAPE); // set text mode to shape
}

void setupSerial() {   
    final String OS = platformNames[platform];
    println("OS "+OS);
   //sensor_data.openPort(OS);    
   
    int speed = 115200;
    String com_port_name;
    
    if(OS == "linux") {
        com_port_name = "/dev/ttyUSB0"; // Linux "/dev/ttyACM#"       
    } else if(OS == "windows") {
        //com_port_name = "COM5:";     /// Pronin E; Alexey please change here 
        com_port_name = "\\\\.\\COM11";   // Windows, COM10 or higher        
    } else if(OS == "macos") { 
        com_port_name = "/dev/cu.usbmodem1217321";        
    } else { 
        com_port_name = Serial.list()[0];    // if you have only ONE serial port active      
        println("Warning! Unknown OS: "+OS);
    }
    
    sensor_port = new Serial(this, com_port_name, speed);  
}


Matrix CreateMatrix(float[][] dat) {
    return  new Matrix(dat);
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  scale += e*5;
  if(scale < 450/200)
      scale = 450/200;
  
}

void SendMagOffset(float x, float y, float z) {
    sensor_port.write(SensorCommands.E_CMD_CODE_SET_MAGNITUDE_OFFSET);            
    sensor_port.write(int(x*float_factor)+" "+int(y*float_factor)+" "+int(z*float_factor)+" ");
}


void SendMagMatrix(Matrix mtx) {
    sensor_port.write(SensorCommands.E_CMD_CODE_SET_MAGNITUDE_MATRIX);
    for (int i=0; i<mtx.M; i++) {
        for (int j=0; j<mtx.N; j++) {
            sensor_port.write((int)(mtx.data[i][j]*float_factor)+" ");
            delay(10);
        }
    }
}

void keyPressed() {        
    byte cmd_code = SensorCommands.E_CMD_CODE_NONE;

    //if(key == 'd' || key == 'D')  // force
    //  cmd_code = E_CMD_CODE_DEBUG_ACTION;
    if (key == 'z' || key == 'Z') { 
        showMessage("Send zero calibration");        
        SendMagOffset(0, 0, 0);
        SendMagMatrix(Matrix.identity(3));
    }

    if (key == '1') {
        showMessage("Send sample calibration");
        SendMagOffset(-72.49, -87.58, 66.34);
        float[][] dat = { {1.001, 0.026, 0.03 }, 
            {0.026, 0.999, -0.001}, 
            {0.003, 0.001, 1.001}
        };
        SendMagMatrix( CreateMatrix( dat ));
    }

    if (key == 'r' || key == 'R') {    
        points_count = 0;        
        is_recording = true;
        showMessage("Start REC");
        return;
    }

    if (key == 'f' || key == 'F') {
        showMessage("Stop REC");
        is_recording = false;
        ProcessData();
        return;
    }    

    if (key == 'a' || key == 'A') {
        showMessage("Apply settings");
        SendSettings();
        return;
    }

    if (key == 's' || key == 'S')
        cmd_code = SensorCommands.E_CMD_CODE_SAVE;
    if (key == 'l' || key == 'L')
        cmd_code = SensorCommands.E_CMD_CODE_LOAD;
    if (key == 'r' || key == 'R')  //recovery
        cmd_code = SensorCommands.E_CMD_CODE_LOAD_DEFAULT;  

    if (cmd_code != SensorCommands.E_CMD_CODE_NONE) {
        println( "Send cmd "+cmd_code);
        sensor_port.write(cmd_code);
    }
}


void CopyDataToResult() {
    for (int i = 0; i< points_count; i++ ) {        
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

void SendSettings() {

    SendMagOffset(mag_offset.x, mag_offset.y, 63.38 - 0.25);
    float[][] d= {
        {mag_matrix.data[0][0], mag_matrix.data[0][1], 0 }, 
        {mag_matrix.data[1][0], mag_matrix.data[1][1], 0}, 
        {0, 0, 1}
    }; 
    SendMagMatrix(CreateMatrix(d));
}

void HardIronCalibration() {       
    Point pmin = new Point(10000, 10000);
    Point pmax = new Point(-10000, -10000);

    for (int i = 0; i< points_result_count; i++ ) {
        Point p = points_result[i];
        pmin.x = min( pmin.x, p.x);
        pmin.y = min( pmin.y, p.y);

        pmax.x = max( pmax.x, p.x);
        pmax.y = max( pmax.y, p.y);
    }

    mag_offset.x = (pmin.x + pmax.x) /2;
    mag_offset.y = (pmin.y + pmax.y) /2;
    applyOffsetToData(-mag_offset.x, -mag_offset.y);
}

void applyOffsetToData(float x, float y) {

    for (int i = 0; i< points_result_count; i++ ) {
        points_result[i].x += x;
        points_result[i].y += y;
    }
}

Point RotateVector(Matrix mtx, Point p) {
    float[][] ar2 = {{p.x, p.y}};
    Matrix m_p = new Matrix(ar2);                
    Matrix m_p2 = m_p.times(mtx);
    return new Point( (float)m_p2.data[0][0], (float)m_p2.data[0][1]);
}

void applyMatrixToData(Matrix mtx) {
    for (int i = 0; i< points_result_count; i++ ) { 
        points_result[i] = RotateVector(mtx, points_result[i]);
    }
}

Point FindMinLenPoint() {
    Point pmin = new Point(100000, 100000);
    float min_len = 100000;

    //find main 
    for (int i = 0; i< points_result_count; i++ ) {
        float len = points_result[i].getLen();
        if (len < min_len) { 
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
    for (int i = 0; i< points_result_count; i++ ) {
        float len = points_result[i].getLen();            
        if (len > max_len) {
            max_len = len;      
            pmax = points_result[i];
        }
    }
    return pmax;
}

Matrix CreateScaleMatrix(float scalex, float scaley) {
    float[][] scale_m_data = {
        {scalex, 0     }, 
        {0, scaley}        
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
    float[][] mat_rot1_data = {
        {cos(angle), sin(angle)}, 
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
    float[][] d = {
        {cos(-angle), sin(-angle)}, 
        {-sin(-angle), cos(-angle)}        
    };
    Matrix mat_rot2 = CreateMatrix(d);   

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

    mag_matrix = mat_final;
}



void drawAxis() {
    float w = width;
    float h = height;
    float len = 5;
    float step = 1;
    stroke(0);
    strokeWeight(1);
    line(0, -h, 0, +h);
    line(-w, 0, w, 0);
    for (float x = -w; x< +w; x+=scale*step ) {
        line(x, -len, x, +len);
    }
    for (float y = -h; y< +h; y+=scale*step ) {
        line(-len, y, len, y);
    }
}


void drawTextIface() {
    float y = 0;
    float x = 10;
    textAlign(LEFT);       
    x = width - 400;
    y = 0;
    textSize(32);
    fill(100, 100, 100);
    text("sensor_data.m_mag.x " + sensor_data.m_mag_raw.x, x, y+=30); 
    //fill(0, 100, 0);
    text("sensor_data.m_mag.y " + sensor_data.m_mag_raw.y, x, y+=30);
    //fill(0, 0, 100);
    text("sensor_data.m_mag.z " + sensor_data.m_mag_raw.z, x, y+=30);


    textSize(32);
    fill(100, 100, 100);
    text("mag_offset_x " + mag_offset.x, x, y+=30); 
    //fill(0, 100, 0);
    text("mag_offset_y " + mag_offset.y, x, y+=30);
    //fill(0, 0, 100);
    //text("mag_offset_y  " + sensor_data.m_magz, x, y+=30);


    float h= 18;
    textSize(h);
    x = 50;
    y = 16;
    text("s - Save ", x, y+=h);
    text("l - Load ", x, y+=h);
    text("z - Zero settings ", x, y+=h);
    text("a - Apply settings ", x, y+=h);
    text("r - Record start ", x, y+=h);
    text("f - Finish recording ", x, y+=h);
    
    fill(255, 128, 128);
    text("Sensor raw  data", x, y+=h);
    fill(128, 255, 128); 
    text("Sensor actual data", x, y+=h);
    fill(128, 128, 255);
    text("Calibration result", x, y+=h);    
    


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
    
    Point p2;
    
        /// after calibrate data

    stroke(128, 128, 255);
    strokeWeight(4);  // Thicker
    for (int i=1; i<points_result_count; i++) {
        Point p1 = points_result[i-1];
        p2 = points_result[i];
        line(p1.x * scale, p1.y * scale, p2.x * scale, p2.y * scale);
    }
    
    p2 = new Point(sensor_data.m_mag_raw.x, sensor_data.m_mag_raw.y); 
    p2.x -= mag_offset.x;
    p2.y -= mag_offset.y;
    p2 = RotateVector(mag_matrix, p2);
    line(0, 0, p2.x * scale, p2.y * scale);
    
    
    // raw data
    stroke(255, 128, 128);
    strokeWeight(4);  // Thicker
    for (int i=1; i<points_count; i++) {
        Point p1 = points[i-1];
        p2 = points[i];
        line(p1.x * scale, p1.y * scale, p2.x * scale, p2.y * scale);
    }
    
    p2 = new Point(sensor_data.m_mag_raw.x, sensor_data.m_mag_raw.y); 
    line(0, 0, p2.x * scale, p2.y * scale);


        
   
    // actual data
    stroke(128, 255, 128);              
    p2 = new Point(sensor_data.m_mag.x, sensor_data.m_mag.y); 
    line(0, 0, p2.x * scale, p2.y * scale);
    
}


void storeData() {
    if (!is_recording)
        return;

    Point p = new Point();
    p.sample_count = 1;
    p.x = sensor_data.m_mag_raw.x;
    p.y = sensor_data.m_mag_raw.y;

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

    if (p2 != null) {       
        int c = p2.sample_count + p.sample_count;
        p2.x = (p2.x * p2.sample_count + p.x * p.sample_count ) / c;
        p2.y = (p2.y * p2.sample_count + p.y * p.sample_count ) / c;
        points[points_count-1] = p2;
    } else {  
        points[points_count] = p;
        points_count++;
        if (points_count >= points.length) {
            showMessage("new lap");      
            for (int i = 0; i < points_count-512; i++) {
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

    if (last_message_timer > 0) 
        last_message_timer -= 0.1;


    //line(mouseX-66, mouseY, mouseX+66, mouseY);
    //line(mouseX, mouseY-66, mouseX, mouseY+66);

    drawTextIface();

    pushMatrix();
    translate(width*0.5, height*0.5);
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
    if(sensor_port == null)
        return;
    int newLine = 13; // new line character in ASCII
    String message;
    do {
        message = sensor_port.readStringUntil(newLine); // read from port until new line
        if (message == null)
            break;

        message = trim(message);        
        if (sensor_data.onMessageSensor(message)) {                 
            //writeLogger();
            storeData();
            continue;
        }

        if (message.length() > 0  ) 
            showMessage(message);        
        //println(message);
    } while (true);
}
