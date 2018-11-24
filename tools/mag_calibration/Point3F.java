final public class Point3F {    
    public float x;
    public float y;
    public float z;
           
    public Point3F() { x=y=z= 0;}
    public Point3F(float _x, float _y, float _z)  { 
        x = _x; y = _y;  z = _z;
    }
    
    public float len() {
        return (float)Math.sqrt( x*x + y*y + z*z );
    }
    
    
    
                
}
