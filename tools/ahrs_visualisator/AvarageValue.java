final public class AvarageValue {    
    public final float[] data;   
    public final int     data_len;
    private      int     m_cursor;
    
    public AvarageValue(int len, float init_value) {
        assert(len > 0);
        this.data_len = len;        
        data = new float[len];
        m_cursor = 0;        
        for(int i=0;i<data_len; i++ ) 
            put(init_value);        
    }
    
    public void put(float val) {
        data[m_cursor] = val;
        m_cursor = (m_cursor + 1) % data_len;        
    }
    public final float getAvg() {
        float sum = 0;
        for(int i=0;i<data_len; i++ ) {
            sum +=data[i];
        }
        return sum / data_len;
    }
    
    public float avg(float val) {
        put(val);
        return getAvg();        
    }
    
                
}
