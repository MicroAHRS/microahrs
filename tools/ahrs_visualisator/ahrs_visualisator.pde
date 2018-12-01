    
import java.util.Date;
import java.text.*;
import processing.serial.*;


float  last_message_time_max = 30.0;
float  last_message_timer = 0.0;
String last_message = "Hello";

SensorData  sensor_data = new SensorData();
Serial      sensor_port;

PrintWriter logger;
boolean logger_first_line = false;
BufferedReader log_reader;

PImage g_cross;
PImage g_cursor;
PImage g_pitch_cursor;
PImage g_yaw_cursor;

void setup()
{
    size(900, 900, P3D);
       
    g_cross = loadImage("res/horizon3.png");    
    g_cursor = loadImage("res/FrontPlaneIcon2.png");
    g_pitch_cursor = loadImage("res/pitch_indicator.png");
    g_yaw_cursor = loadImage("res/yaw_indicator.png");
      
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

void drawTextCommand() {
    float x = width/2 - 150;
    float y = 120;
    float h= 14;
    textSize(h = 14);
    fill(0, 0, 0);

    text("q - RESET_PITCH_ROLL ", x, y+=h );
    text("c - CALIBRATE_GYRO ", x, y+=h );         
    text("b - CHANGE_BETA ", x, y+=h );
    text("z - CHANGE_ZETA ", x, y+=h );
    text("n - CHANGE_NETA ", x, y+=h );       
    text("f - BOOST_FILTER ", x, y+=h );
    y+=h/2;
    text("g - TOGGLE_GYRO ", x, y+=h );
    text("m - TOGGLE_MAG ", x, y+=h );    
    text("a - TOGGLE_ACC ", x, y+=h );
    text("w - DEBUG_ACTION ", x, y+=h );
    //text("v - VERBOSE OUTPUT ", x, y+=h );
    y+=h/2;

    x = width/2 + 50;
    y = 120;

    text("s - SAVE ", x, y+=h );
    text("l - LOAD ", x, y+=h );
    text("d - LOAD_DEFAULT ", x, y+=h );
    y+=h/2;
    //text("p - SET_PITCH_AND_ROLL", x, y+=h );    
    //text("y - SET_YAW ", x, y+=h );
    //text("i - DEFAULT_ORIENTATION ", x, y+=h );
    //y+=h/2;
    text("r - RECORDING START ", x, y+=h );
    text("o - OPEN RECORD ", x, y+=h );    
    y+=h/2;
    //text("r - SensorCommands.E_CMD_CODE_LOAD_DEFAULT ", x, y+=h );
}

void showMessage(String message) 
{
    last_message = message;
    last_message_timer = last_message_time_max;
}

void onCmdRecordStart() {

    if (logger != null) {        
        logger.flush(); // Writes the remaining data to the file
        logger.close(); // Finishes the file
        logger = null;
        showMessage("Record stop");
        return;
    }

    DateFormat formatter = new SimpleDateFormat("yyyy-MM-dd_HH:mm:ss");
    Date d = new Date();    
    showMessage("Record start");
    logger = createWriter("logs/log_"+formatter.format(d)+".csv");
    logger_first_line = true;
}



void cmdOpenLog() {
    selectInput("Select a file to process:", "fileSelected");
}

void fileSelected(File selection) {
    if (selection == null) {
        log_reader = null;
        return;
    }

    log_reader = createReader(selection.getAbsolutePath());
}
   
void SendCmd(byte cmd) {
    println( "Send cmd "+cmd);
    sensor_port.write(cmd);               
}

void keyPressed() {
    println("Key = " + int(key));
    byte cmd_code = SensorCommands.E_CMD_CODE_NONE;
    if (key == 'w' || key == 'W')  // force
        cmd_code = SensorCommands.E_CMD_CODE_DEBUG_ACTION;
    if (key == 'c' || key == 'C')  // force
        cmd_code = SensorCommands.E_CMD_CODE_CALIBRATE_GYRO;
    if (key == 'q' || key == 'Q')  // force
        cmd_code = SensorCommands.E_CMD_CODE_RESET_PITCH_ROLL;
    if (key == 'f' || key == 'F')  // force
        cmd_code = SensorCommands.E_CMD_CODE_BOOST_FILTER;
    if (key == 'z' || key == 'Z')
        cmd_code = SensorCommands.E_CMD_CODE_CHANGE_ZETA;
    if (key == 'b' || key == 'B')
        cmd_code = SensorCommands.E_CMD_CODE_CHANGE_BETA;
    if (key == 'n' || key == 'N')
        cmd_code = SensorCommands.E_CMD_CODE_CHANGE_NETA;
    if (key == 'm' || key == 'M')
        cmd_code = SensorCommands.E_CMD_CODE_TOGGLE_MAG;
    if (key == 'a' || key == 'A')
        cmd_code = SensorCommands.E_CMD_CODE_TOGGLE_ACC;
    if (key == 'g' || key == 'G')
        cmd_code = SensorCommands.E_CMD_CODE_TOGGLE_GYRO;
    if (key == 's' || key == 'S')
        cmd_code = SensorCommands.E_CMD_CODE_SAVE;
    if (key == 'l' || key == 'L')
        cmd_code = SensorCommands.E_CMD_CODE_LOAD;
    if (key == 'd' || key == 'D')  //recovery
        cmd_code = SensorCommands.E_CMD_CODE_LOAD_DEFAULT;  
    //if (key == 'y' || key == 'Y')  //yaw
    //    cmd_code = SensorCommands.E_CMD_CODE_SET_YAW_NORTH;
    //if (key == 'p' || key == 'P')  // pitch
    //    cmd_code = SensorCommands.E_CMD_CODE_SET_GRAVITY_VECTOR;
    if (key == 'i' || key == 'I')  // pitch
        cmd_code = SensorCommands.E_CMD_CODE_DEFAULT_ORIENTATION;
    if (key == 'v' )
        cmd_code = SensorCommands.E_CMD_CODE_TOGGLE_PRINT_MODE;
    if (key == 'o' )
        cmdOpenLog();    
    if (key == 'r' || key == 'R')   
        onCmdRecordStart();   
  
    if (cmd_code != SensorCommands.E_CMD_CODE_NONE)
        SendCmd(cmd_code);    
}

String getAngleMark(int angle) {
    if(angle == 0)
        return "СЕВЕР";
    if(angle == 45)
        return "СВ";
    if(angle == 90)
        return "ВОСТОК";
    if(angle == 135)
        return "ЮВ";
    if(angle  == 180)
        return "ЮГ";
    if(angle  == 225)
        return "ЮЗ";
    if(angle  == 270)
        return "ЗАПАД";
    if(angle  == 315)
        return "СЗ";
    return "";
}

void drawHorisont(float roll, float pitch, float yaw, float x, float y) {

    pushMatrix(); // begin object
    translate(x, y, -100);
  
    rotate(radians(-roll));

    float pixels_per_degree = 10;
    translate(0, pitch * pixels_per_degree);

    pushMatrix(); // begin object
    stroke(0);
    fill(0, 0, 0); // ground       
    box(2400 * 2, 1, 2);
    if(sensor_data.m_beta == 0 && sensor_data.m_zeta == 0)
        fill(206, 130, 16); // ground
    else
        fill(128, 255, 128); // ground
    translate(0, 600 * 2); // set position to edge of Arduino box
    box(2400 * 2, 1200 * 2, 1);
    popMatrix(); // end of object

    translate(0, 0, 10); 
    noStroke(); 
    fill(0, 0, 0); 
    float h = 100;
    textAlign(CENTER);
    float angle_step = 15;    
    for (int ya = -360; ya <= +360 + 360; ya += angle_step) {
        float lx = (-yaw + ya) * pixels_per_degree;      
        h=(ya % 45 == 0) ? 70 : 50;
        h=(ya % 90 == 0) ? 100 : h;
        float w=(ya % 45 == 0) ? 2 : 1;        
        
          
        pushMatrix();
        translate(lx, 0, 10);
        
        String mark = getAngleMark((ya + 360) % 360);
        if(mark!= "")
            text(mark, 0, -h*1.5);
      
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
    text("roll    " + sensor_data.m_angles.x, x, y+=30);     
    fill(0, 100, 0);
    text("pitch " + sensor_data.m_angles.y, x, y+=30);
    fill(0, 0, 100);
    text("yaw   " + sensor_data.m_angles.z, x, y+=30);

    textSize(24);
    fill(100, 0, 100);
    text("roll err  " + nf(sensor_data.m_gerr.x, 1, 5), x, y+=30);     
    text("pitch err " + nf(sensor_data.m_gerr.y, 1, 5), x, y+=24);    
    text("yaw err   " + nf(sensor_data.m_gerr.z, 1, 5), x, y+=24);

    y+=24;
    fill(100, 100, 0);
    text("beta " + nf(sensor_data.m_beta, 1, 5), x, y+=24);     
    text("zeta " + nf(sensor_data.m_zeta, 1, 5), x, y+=24);
    text("neta " + nf(sensor_data.m_neta, 1, 5), x, y+=24);


    x = width - 200;
    y = 0;
    textSize(32);
    fill(100, 100, 100);
    text("mag.x " + sensor_data.m_mag_raw.x, x, y+=30); 
    //fill(0, 100, 0);
    text("mag.y " + sensor_data.m_mag_raw.y, x, y+=30);
    //fill(0, 0, 100);
    text("mag.z " + sensor_data.m_mag_raw.z, x, y+=30);

    fill(100, 100, 100);
    text("acc.x " + sensor_data.m_acc.x, x, y+=30); 
    //fill(0, 100, 0);
    text("acc.y " + sensor_data.m_acc.y, x, y+=30);
    //fill(0, 0, 100);
    text("acc.z " + sensor_data.m_acc.z, x, y+=30);


    fill(100, 100, 0);
    text("g " + nf(sensor_data.m_g, 1, 2), x, y+=30);
    //text("a " +  nf(g_anlge_roll,1,2), x, y+=30);
    text("overload " + nf(sensor_data.m_overload, 1, 1), x, y+=30);

    x = width/2 - 100;
    y = 0;
    fill(100, 0, 0);
    text("temp " + sensor_data.m_temp, x, y+=30);
    text("fps " + sensor_data.m_fps, x, y+=30);     
    text("time " + sensor_data.m_time, x, y+=30);    

    drawTextCommand();

    textSize(24);
    x = width / 2;
    y = height /2 - 100;
    float alpa = last_message_timer / last_message_time_max * 255; 
    fill(128, 0, 0, alpa);
    textAlign(CENTER);
    text(last_message, x, y+=14 );
}

void drawCursor(float x, float y, float angle, PImage img, boolean flip, float anlge_aim, boolean rev) {
    pushMatrix(); // begin object
    translate(x, y);
    rotate(radians(angle));  
    fill(200,200,200,128);
    float w2 = img.width / 2;
    float h2 = img.height / 2;
    float d = 260;
    ellipseMode(CENTER);
    ellipse(0, 0, d, d);
    if (flip)
        scale(1, -1);
    image(img, -w2, -h2);  

    float d2;
    //draw aim backgroud
    pushMatrix(); // begin aim indicator  
    if (flip)
        scale(1, -1);
    rotate(radians(-angle));    
    //boolean flip2 = flip? !rev : rev;
    //translate(0, (rev?-1:1)*250 / 2);
    translate(0, (rev?-1:1)*250 / 2);
    fill(128, 128, 128);
    d2 = 16;
    ellipseMode(CENTER);
    ellipse(0, 0, d2, d2);   
    popMatrix(); // end of object

    //draw aim
    pushMatrix(); // begin aim indicator  
    rotate(radians(-anlge_aim));
    translate(0, (rev?-1:1)*250 / 2);
    fill(128, 255, 128);
    d2 = 10;
    ellipseMode(CENTER);
    ellipse(0, 0, d2, d2);   
    popMatrix(); // end of object 

    popMatrix(); // end of object

    textAlign(CENTER);
    textSize(32);
    fill(0, 0, 0);
    text(nf(angle, 3, 2), x, y+= (d/2 + 32));
}

void drawGravityIndicator(float x, float y, float angle, float d) 
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
    readLogEvent();
    background(255); // set background to white
    lights();

    if (last_message_timer > 0) 
        last_message_timer -= 0.1;

    drawHorisont(sensor_data.m_angles.x, sensor_data.m_angles.y, sensor_data.m_angles.z, width*0.5, height*0.5);
    stroke(0);

    drawCursor(width*0.5, height*0.75, sensor_data.m_angles.x, g_cursor, false,  sensor_data.m_g_anlge_roll, false);
    drawCursor(width*0.80, height*0.75, sensor_data.m_angles.y, g_pitch_cursor, sensor_data.m_angles.x > 90 || sensor_data.m_angles.x < - 90,  sensor_data.m_g_angle_pitch, false);  
    drawCursor(width*0.20, height*0.75, sensor_data.m_angles.z, g_yaw_cursor, false,  sensor_data.m_mag_angle_yaw, false);

    image(g_cross, width*0.5 -g_cross.width /2, height*0.5 -g_cross.height * 0);
    fill(200, 200, 200);
    drawGravityIndicator(width*0.5, height*0.5, 0, 40);
    fill(100, 200, 200);
    drawGravityIndicator(width*0.5, height*0.5,  - sensor_data.m_g_anlge_roll, 20);
    drawTextIface(); 
}

void writeLogger() {

    if (logger==null)
        return;

    if (logger_first_line) {
        logger.println(sensor_data.makeMessageRecordHeader());
        logger_first_line = false;
    }
  
    logger.println( sensor_data.makeMessageRecord());
}

void readLogEvent() {
    if (log_reader == null)
        return;
    try {
        String message = log_reader.readLine();
        message = log_reader.readLine();
        if (message == null) {
            log_reader.close();
            log_reader = null;
            showMessage("Finished"); 
            return;
        }                             
        sensor_data.onMessageRecord(message);
    } 
    catch (IOException e) {
        e.printStackTrace();
    }
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
            writeLogger();
            continue;
        }

        if (message.length() > 0  ) 
            showMessage(message);        
        //println(message);
    } while (true);
}
