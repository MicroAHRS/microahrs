import processing.serial.*;
import java.util.Date;
import java.text.*;

Serial myPort;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

Point3F acc = new Point3F();
Point3FAvarage acc_avg = new Point3FAvarage(10);
Point3FAvarage mag_avg = new Point3FAvarage(10);
Point3FAvarage gerr_avg = new Point3FAvarage(10);

Point3F gerr = new Point3F();
Point3F gyro = new Point3F();
Point3F mag = new Point3F();
Point3F mag_raw = new Point3F();

boolean boost_at_start = true;

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

String time;

PrintWriter logger;
boolean logger_first_line = false;

BufferedReader log_reader;


byte
    E_CMD_CODE_NONE = 0, 
    E_CMD_CODE_RESET_PITCH_ROLL      = 1, // сбросить крен тангаж
    E_CMD_CODE_SET_YAW_BY_MAG        = 2, // установить углы по магнетометру
    E_CMD_CODE_SET_PITCH_ROLL_BY_ACC = 3, // установить углы по акселерометру
    E_CMD_CODE_BOOST_FILTER          = 4, // установить углы по акселерометру

    E_CMD_CODE_CHANGE_BETA           = 5, 
    E_CMD_CODE_CHANGE_ZETA           = 6, 

    E_CMD_CODE_SET_GRAVITY_VECTOR  = 10, // текущее направление силы тяжести принять за 0 (roll pitch)
    E_CMD_CODE_SET_YAW_NORTH       = 11, // текущее направление на север принять за 0 (yaw)
    E_CMD_CODE_DEFAULT_ORIENTATION = 12, // сбросить модификатор ориентации

    E_CMD_CODE_CALIBRATE_GYRO       = 20, 
    E_CMD_CODE_SET_MAGNITUDE_OFFSET = 21, 
    E_CMD_CODE_SET_MAGNITUDE_MATRIX = 22, 

    E_CMD_CODE_SET_ACC_OFFSET       = 23, 
    E_CMD_CODE_SET_ACC_SCALE        = 24, 

    E_CMD_CODE_DEBUG_ACTION         = 30, 
    E_CMD_CODE_TOGGLE_GYRO          = 31, 
    E_CMD_CODE_CALIBRATION_STOP     = 32, // code = space - useful when send calibration
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

    g_cross = loadImage("res/horizon3.png");
    //g_cursor = loadImage("cursor.png");  
    g_cursor = loadImage("res/FrontPlaneIcon2.png");
    g_pitch_cursor = loadImage("res/pitch_indicator.png");
    g_yaw_cursor = loadImage("res/yaw_indicator.png");
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
    float y = 120;
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
    text("w - DEBUG_ACTION ", x, y+=h );
    text("v - VERBOSE OUTPUT ", x, y+=h );
    y+=h/2;

    x = width/2 + 50;
    y = 120;

    text("s - SAVE ", x, y+=h );
    text("l - LOAD ", x, y+=h );
    text("d - LOAD_DEFAULT ", x, y+=h );
    y+=h/2;
    text("p - SET_PITCH_AND_ROLL", x, y+=h );    
    text("y - SET_YAW ", x, y+=h );
    text("i - DEFAULT_ORIENTATION ", x, y+=h );
    y+=h/2;
    text("r - RECORDING START ", x, y+=h );        
    y+=h/2;


    //text("r - E_CMD_CODE_LOAD_DEFAULT ", x, y+=h );
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

void sendCmdBoost() {
    myPort.write(E_CMD_CODE_BOOST_FILTER);
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

void keyPressed() {
    println("Key = " + int(key));
    int keyIndex = -1;  

    byte cmd_code = E_CMD_CODE_NONE;

    if (key == 'w' || key == 'W')  // force
        cmd_code = E_CMD_CODE_DEBUG_ACTION;
    if (key == 'c' || key == 'C')  // force
        cmd_code = E_CMD_CODE_CALIBRATE_GYRO;
    if (key == 'q' || key == 'Q')  // force
        cmd_code = E_CMD_CODE_RESET_PITCH_ROLL;
    if (key == 'f' || key == 'F')  // force
        cmd_code = E_CMD_CODE_BOOST_FILTER;
    if (key == 'z' || key == 'Z')
        cmd_code = E_CMD_CODE_CHANGE_ZETA;
    if (key == 'b' || key == 'B')
        cmd_code = E_CMD_CODE_CHANGE_BETA;
    if (key == 'm' || key == 'M')
        cmd_code = E_CMD_CODE_TOGGLE_MAG;
    if (key == 'a' || key == 'A')
        cmd_code = E_CMD_CODE_TOGGLE_ACC;
    if (key == 'g' || key == 'G')
        cmd_code = E_CMD_CODE_TOGGLE_GYRO;
    if (key == 's' || key == 'S')
        cmd_code = E_CMD_CODE_SAVE;
    if (key == 'l' || key == 'L')
        cmd_code = E_CMD_CODE_LOAD;
    if (key == 'd' || key == 'D')  //recovery
        cmd_code = E_CMD_CODE_LOAD_DEFAULT;  
    if (key == 'y' || key == 'Y')  //yaw
        cmd_code = E_CMD_CODE_SET_YAW_NORTH;
    if (key == 'p' || key == 'P')  // pitch
        cmd_code = E_CMD_CODE_SET_GRAVITY_VECTOR;
    if (key == 'i' || key == 'I')  // pitch
        cmd_code = E_CMD_CODE_DEFAULT_ORIENTATION;
    if (key == 'v' )
        cmd_code = E_CMD_CODE_TOGGLE_PRINT_MODE;
    if (key == 'o' )
        cmdOpenLog();    

    if (key == 'r' || key == 'R') {      
        onCmdRecordStart();   
        return;
    }

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
    //  acc.z = 0.6124513;
    //  myPort.write(acc.x + " " + acc.y + " "+ acc.z); 
    //  return;
    //}


    if (cmd_code != E_CMD_CODE_NONE) {
        println( "Send cmd "+cmd_code);
        myPort.write(cmd_code);
    }
}



void drawObject2(float roll, float pitch, float yaw, float x, float y) {

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
    translate(0, pitch * pixels_per_degree);


    pushMatrix(); // begin object
    stroke(0);
    fill(0, 0, 0); // ground       
    box(2400 * 2, 1, 2);

    fill(128, 255, 128); // ground
    translate(0, 600 * 2); // set position to edge of Arduino box
    box(2400 * 2, 1200 * 2, 1);
    popMatrix(); // end of object

    translate(0, 0, 10); 
    noStroke(); 
    fill(0, 0, 0); 
    float h = 100;
    float angle_step = 10;    
    for (int ya = -360; ya <= +360 + 360; ya += angle_step) {
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
    text("roll err  " + nf(gerr.x, 1, 5), x, y+=30);     
    text("pitch err " + nf(gerr.y, 1, 5), x, y+=24);    
    text("yaw err   " + nf(gerr.z, 1, 5), x, y+=24);

    y+=24;
    fill(100, 100, 0);
    text("beta " + nf(beta, 1, 5), x, y+=24);     
    text("zeta " + nf(zeta, 1, 5), x, y+=24);        


    x = width - 200;
    y = 0;
    textSize(32);
    fill(100, 100, 100);
    text("mag.x " + mag_raw.x, x, y+=30); 
    //fill(0, 100, 0);
    text("mag.y " + mag_raw.y, x, y+=30);
    //fill(0, 0, 100);
    text("mag.z " + mag_raw.z, x, y+=30);

    fill(100, 100, 100);
    text("acc.x " + acc.x, x, y+=30); 
    //fill(0, 100, 0);
    text("acc.y " + acc.y, x, y+=30);
    //fill(0, 0, 100);
    text("acc.z " + acc.z, x, y+=30);


    fill(100, 100, 0);
    text("g " + nf(g, 1, 2), x, y+=30);
    //text("a " +  nf(g_anlge_roll,1,2), x, y+=30);
    text("overload " + nf(overload, 1, 1), x, y+=30);

    x = width/2 - 100;
    y = 0;
    fill(100, 0, 0);
    text("temp " + temp, x, y+=30);
    text("fps " + fps, x, y+=30);     
    text("time " + time, x, y+=30);    

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
    fill(200);
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
    translate(0, (rev?1:1)*250 / 2);
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

    drawObject2(roll, pitch, yaw, width*0.5, height*0.5);
    stroke(0);

    drawCursor(width*0.5, height*0.75, roll, g_cursor, false, g_anlge_roll, false);
    drawCursor(width*0.80, height*0.75, pitch, g_pitch_cursor, roll > 90 || roll < - 90, g_angle_pitch, false);  
    drawCursor(width*0.20, height*0.75, yaw, g_yaw_cursor, false, mag_angle_yaw, true);

    image(g_cross, width*0.5 -g_cross.width /2, height*0.5 -g_cross.height * 0);
    fill(200, 200, 200);
    drawGravityIndicator(width*0.5, height*0.5, 0, 40);
    fill(100, 200, 200);
    drawGravityIndicator(width*0.5, height*0.5, -g_anlge_roll, 20);
    drawTextIface();
}

Matrix getRollMatrix(float a) 
{    
    double[][] d = { 
        {1, 0, 0}, 
        {0, cos(a), -sin(a)}, 
        {0, sin(a), cos(a)}       
    };
    return new Matrix(d);
}

Matrix getPitchMatrix(float a) 
{    
    double[][] d = { 
        {cos(a), 0, sin(a)}, 
        {0, 1, 0}, 
        {-sin(a), 0, cos(a)}       
    };
    return new Matrix(d);
}


void RotateMag() {
    double[][] d = {{mag.x, mag.y, mag.z}};
    Matrix mag_m = new Matrix(d);

    //Matrix m = getPitchMatrix(radians(pitch)).times(getRollMatrix(radians(roll)));
    Matrix m = getRollMatrix(radians(-roll)).times(getPitchMatrix(radians(pitch)));
    Matrix r = mag_m.times(m);
    mag.x = (float)r.data[0][0];
    mag.y = (float)r.data[0][1];
    mag.z = (float)r.data[0][2];
}


void writeLogger() {

    if (logger==null)
        return;

    if (logger_first_line) {
        logger.println("time, timestamp,roll,pitch,yaw,acc.x,acc.y,acc.z ,temp ,gerr.x,gerr.y ,gerr.z,beta,zeta,mag.x,mag.y,mag.z, fps, ,mag_raw.x,mag_raw.y,mag_raw.z,g ,g_anlge_roll ,g_angle_pitch,mag_angle_yaw");
        logger_first_line = false;
    }

    DateFormat formatter = new SimpleDateFormat("yyyy-MM-dd_HH_mm_ss");
    Date d = new Date();                          
    logger.println( formatter.format(d) + "," + d.getTime() / 1000
        + "," + roll + "," + pitch + "," + yaw  
        + "," + acc.x + "," + acc.y + "," + acc.z 
        + "," + temp 
        + "," + gerr.x + "," + gerr.y  + "," + gerr.z
        + "," + beta + "," + zeta          
        + "," + mag.x + "," + mag.y  + "," + mag.z          
        + "," + fps
        + "," + mag_raw.x + "," + mag_raw.y  + "," + mag_raw.z
        + "," + g 
        + "," + g_anlge_roll  + "," + g_angle_pitch + "," + mag_angle_yaw           
        );
}

boolean onDataMessage(String message, String[] list , boolean from_sensor) {
    if (list.length <= 4)
        return false;
        
    if(from_sensor && !list[0].equals("Orient:"))
        return false;
        
    float FLOAT_FACKTOR = from_sensor ? 1000 : 1;
    int l = 0;
    if(from_sensor)
        l++; 
    
    if(from_sensor) {
        DateFormat formatter = new SimpleDateFormat("HH:mm:ss");
        Date d = new Date();    
        time = formatter.format(d);
    }

    if (list.length >= l+3) {
        roll   = float(list[l++]); // convert to float roll
        pitch  = float(list[l++]); // convert to float pitch
        yaw    = float(list[l++]) ; // convert to float yaw
    }
    
    if(from_sensor) {
        //roll  *= -1;
        pitch *= -1;
        yaw   *= -1;
    }else {
        yaw   *= -1;
    }
    
    if(from_sensor)  // acc
        l++;
    if (list.length >= l+3) {
        acc.x = float(list[l++]) ; 
        acc.y = float(list[l++]) ; 
        acc.z = float(list[l++]) ;
    }
    if(from_sensor) // temp
        l++;
    temp = float(list[l++]);
    
    if(from_sensor) // gerr
        l++;
    if (list.length >= l+3) {    
        gerr.x = float(list[l++]) / FLOAT_FACKTOR;
        gerr.y = float(list[l++]) / FLOAT_FACKTOR;
        gerr.z = float(list[l++]) / FLOAT_FACKTOR;
    }

    if (list.length >= l+2) {
        beta = float(list[l++]) / FLOAT_FACKTOR;
        zeta = float(list[l++]) / FLOAT_FACKTOR;
    }
    
    if(from_sensor)  // mag
        l++;
    if (list.length >= l+3) {
        mag.x = float(list[l++]) ; 
        mag.y = float(list[l++]) ; 
        mag.z = float(list[l++]) ;
    }

    if(from_sensor)
        l++;
    if (list.length >= l) 
        fps = float(list[l++]) ;     

    if(from_sensor)
        l++;
    if (list.length >= l+3) {        
        mag_raw.x = float(list[l++]) / FLOAT_FACKTOR;
        mag_raw.y = float(list[l++]) / FLOAT_FACKTOR;
        mag_raw.z = float(list[l++]) / FLOAT_FACKTOR;
    }

    if (yaw <0)
        yaw += 360;


    acc = acc_avg.avg(acc);
    mag = mag_avg.avg(mag);
    gerr = gerr_avg.avg(gerr);

    g= acc.len();
    g_anlge_roll = degrees(atan2( acc.y, acc.z));
    g_angle_pitch = degrees(atan2( acc.x, acc.z));

    RotateMag();
    mag_angle_yaw = degrees(atan2( mag.y, mag.x));
    overload = g / 9.8;


    if (boost_at_start) {
        boost_at_start = false;
        sendCmdBoost();
    }

    writeLogger();   
    return true;
}

void serialEvent()
{
    if(myPort == null)
        return;
    int newLine = 13; // new line character in ASCII
    String message;
    do {
        message = myPort.readStringUntil(newLine); // read from port until new line
        if (message == null)
            break;

        message = trim(message);
        String[] list = split(message, " ");

        if (onDataMessage(message, list, true)) {
            
            continue;
        }


        if (message.length() > 0  ) 
            showMessage(message);        
        println(message);
    } while (true);
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
        //print(message);
        String[] list = split(message, ",");
        time = list[0];
        String[] list2 = subset(list, 2);        
        onDataMessage(message, list2, false);
    } 
    catch (IOException e) {
        e.printStackTrace();
    }
}
