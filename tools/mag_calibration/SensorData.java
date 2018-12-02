import java.util.Date;
import java.text.*;
import processing.core.PApplet;


final public class SensorData {    
    Point3F m_angles = new Point3F();
    Point3F m_acc = new Point3F();       
    Point3F m_gerr = new Point3F();
    //Point3F m_gyro = new Point3F();
    Point3F m_mag = new Point3F();
    Point3F m_mag_raw = new Point3F();
    Point3F m_north_local = new Point3F();
    
    
    Point3FAvarage m_acc_avg = new Point3FAvarage(2);
    Point3FAvarage m_mag_avg = new Point3FAvarage(2);
    Point3FAvarage m_gerr_avg = new Point3FAvarage(2);
    AvarageValue   m_g_angle_roll_avg = new AvarageValue(10,0);    
    AvarageValue   m_mag_err_avg = new AvarageValue(10,0);
    
    //Point3FAvarage m_mag_err_avg = new Point3FAvarage(2);  
    
    int    m_is_moving;
           
    float  m_temp = 0;    
    float  m_beta = 0;
    float  m_zeta = 0;
    float  m_neta = 0;

    float  m_g=0;
    float  m_g_anlge_roll=0;
    float  m_g_angle_pitch=0;
    float  m_mag_angle_yaw=0;
    float  m_overload=0;    
    float  m_fps= 0;
    String m_time;
    
    
    Matrix m_orient_mtx;
    
    
    SensorData() {
        float[][] d = { 
            {-1, 0, 0}, 
            {0, -1, 0}, 
            {0, 0, 1}       
        };
        m_orient_mtx = new Matrix(d);
    }
        
        
    private float FLOAT_FACKTOR = 1;
        
    private int readAngles(String[] list, int l) {
        if (list.length >= l+3) {
            m_angles.x  = Float.parseFloat(list[l++]); // convert to float roll
            m_angles.y  = Float.parseFloat(list[l++]); // convert to float pitch
            m_angles.z  = Float.parseFloat(list[l++]); // convert to float yaw
        }
        return l;
    }
    
    private int readAcc(String[] list, int l) {   
         if (list.length >= l+3) {
            m_acc.x = Float.parseFloat(list[l++]) ; 
            m_acc.y = Float.parseFloat(list[l++]) ; 
            m_acc.z = Float.parseFloat(list[l++]) ;
        }
        return l;
    }
    private int readTemp(String[] list, int l) {   
        if (list.length >= l+1) 
            m_temp = Float.parseFloat(list[l++]);
        return l;
    }
    private int readGyroErr(String[] list, int l) {  
         if (list.length >= l+3) {    
            m_gerr.x = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_gerr.y = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_gerr.z = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
        }
        return l;
    }
    private int readBetaZeta(String[] list, int l) { 
        if (list.length >= l+3) {
            m_beta = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_zeta = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_neta = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
        }
        return l;
    }
    
    private int readMag(String[] list, int l) {    
        if (list.length >= l+3) {
            m_mag.x = Float.parseFloat(list[l++]) ; 
            m_mag.y = Float.parseFloat(list[l++]) ; 
            m_mag.z = Float.parseFloat(list[l++]) ;
        }
        return l;
    }
    private int readMagRaw(String[] list, int l) { 
        if (list.length >= l+3) {        
            m_mag_raw.x = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_mag_raw.y = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_mag_raw.z = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
        }
        return l;
    }
    
    private int readNorthLocal(String[] list, int l) { 
        if (list.length >= l+3) {        
            m_north_local.x = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_north_local.y = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
            m_north_local.z = Float.parseFloat(list[l++]) / FLOAT_FACKTOR;
        }
        return l;
    }

    private int readFPS(String[] list, int l) {           
        if (list.length >= l+1) 
            m_fps = Float.parseFloat(list[l++]) ;             
        return l;
    }
    
    private int read(String[] list, int l) {        
        return l;
    }

    private void readCurTime() {        
        DateFormat formatter = new SimpleDateFormat("HH:mm:ss");
        Date d = new Date();    
        m_time = formatter.format(d);
    }

    public boolean onMessageOrient(String[] list) {
        FLOAT_FACKTOR = 1000;
        int l = 0;
        
        l++; // Orient
                              
        readCurTime();        
        l = readAngles(list, l);
        
        l++;
        l = readAcc(list, l);
                  
        l++;
        l = readTemp(list, l);
        
        l++;
        l = readGyroErr(list, l);       
        l = readBetaZeta(list, l);
        ///////////  Magnitometer ////////////               
        l++;
        l = readMag(list, l);
        ///////////// FPS ////////////
        l++;
        l = readFPS(list, l);
        ///////////// MAG RAW //////////////
        l++;
        l = readMagRaw(list, l);
        l++;
        l = readNorthLocal(list, l);
    
        
        m_angles.y *= -1;
        m_angles.z *= -1; 
        m_gerr.x *= -1;
        m_gerr.y *= -1;
        
        m_angles.z -=180;
          
        while (m_angles.z <0)
            m_angles.z += 360;
        while (m_angles.z >=360)
            m_angles.z -= 360;
    
                
        m_angles = m_orient_mtx.rotatePoint(m_angles);
        m_acc    = m_orient_mtx.rotatePoint(m_acc);        
        m_gerr   = m_orient_mtx.rotatePoint(m_gerr);
        
                   
        onDataCanged();
        return true;
    }
    
    private void onDataCanged() {
        m_acc  = m_acc_avg.avg(m_acc);
        m_mag  = m_mag_avg.avg(m_mag);
        m_gerr = m_gerr_avg.avg(m_gerr);
         
        m_g = m_acc.len();
        m_g_anlge_roll  = processing.core.PApplet.degrees((float)Math.atan2( m_acc.y, m_acc.z));
        m_g_angle_pitch = processing.core.PApplet.degrees((float)Math.atan2( m_acc.x, m_acc.z));            
        m_overload = m_g / 9.8f;
        
        //Matrix m = Matrix.GetPitchMatrix(processing.core.PApplet.radians(-m_angles.x)).times(
        //                Matrix.GetRollMatrix(processing.core.PApplet.radians(m_angles.y))
        //                );
        //Matrix m = Matrix.GetPitchMatrix(processing.core.PApplet.radians(-m_angles.x));
        //m_north_local = m.rotatePoint(m_north_local);
        //m_north_local = m_mag_err_avg.avg(m_north_local);
        float mag_err = processing.core.PApplet.degrees((float)Math.atan2(m_north_local.y, m_north_local.x)); 
        mag_err = m_mag_err_avg.avg(mag_err);
        m_mag_angle_yaw = m_angles.z + mag_err;        
                
        m_g_angle_roll_avg.avg(m_g_anlge_roll);        
    }

    public boolean onMessageSensor(String message) {
        String[] list = message.split(" "); 
        if (list.length <= 0)
            return false;
            
        if(list[0].equals("Orient:"))
            return onMessageOrient(list);
                            
        return false;
    }
    
    public boolean onMessageRecord(String message) {
        String[] list = message.split(",");        
        FLOAT_FACKTOR =1;
        int l = 0;
        if (list.length >= l+1)     
            m_time = list[0];
        l++; // skip timestamp
                                 
        l = readAngles(list, l);                
        l = readAcc(list, l);                        
        l = readTemp(list, l);       
        l = readGyroErr(list, l);       
        l = readBetaZeta(list, l);
        l = readMag(list, l);
        l = readFPS(list, l);
        l = readMagRaw(list, l);
        onDataCanged();
        return true;
    }
    
    public String makeMessageRecordHeader() {
        return "time, timestamp" 
             +",roll,pitch,yaw"
             +",acc.x,acc.y,acc.z"
             +",temp ,gerr.x,gerr.y,gerr.z"
             +",beta,zeta,neta"
             +",mag.x,mag.y,mag.z"
             +",fps"
             +",mag_raw.x,mag_raw.y,mag_raw.z"
             +",g"
             +",g_anlge_roll,g_angle_pitch,mag_angle_yaw";
    } 
    
    public String makeMessageRecord() {
        DateFormat formatter = new SimpleDateFormat("yyyy-MM-dd_HH_mm_ss");
        Date d = new Date();                          
        String msg = formatter.format(d) + "," + d.getTime() / 1000
            + "," + m_angles.x + "," + m_angles.y + "," + m_angles.z  
            + "," + m_acc.x + "," + m_acc.y + "," + m_acc.z 
            + "," + m_temp 
            + "," + m_gerr.x + "," + m_gerr.y  + "," + m_gerr.z
            + "," + m_beta + "," + m_zeta + "," + m_neta          
            + "," + m_mag.x + "," +  m_mag.y  + "," +  m_mag.z          
            + "," + m_fps
            + "," + m_mag_raw.x + "," + m_mag_raw.y  + "," + m_mag_raw.z
            + "," + m_g 
            + "," + m_g_anlge_roll  + "," + m_g_angle_pitch + "," + m_mag_angle_yaw           
            ;
         return msg; 
    }
    
    
    private void rotateMag() {
        // може стоит наоборот - умножение с лева на право
        Matrix m = Matrix.GetPitchMatrix(processing.core.PApplet.radians(-m_angles.x)).times(
                        Matrix.GetRollMatrix(processing.core.PApplet.radians(m_angles.y))
                        );
        m_mag = m.rotatePoint(m_mag);  
    }
}   
                
