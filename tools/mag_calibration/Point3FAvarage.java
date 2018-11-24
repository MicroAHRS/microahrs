final public class Point3FAvarage {
    private AvarageValue x;
    private AvarageValue y;
    private AvarageValue z;
       
           
    public Point3FAvarage(int len) {       
        x = new AvarageValue(len, 0 );
        y = new AvarageValue(len, 0 );
        z = new AvarageValue(len, 0 );
    }
       
    public Point3F getAvg() {
        return new Point3F( x.getAvg(), y.getAvg(), z.getAvg());
    }    
    
    public Point3F avg(Point3F p ) {
        return new Point3F( x.avg(p.x), y.avg(p.y), z.avg(p.z));
    }
}
