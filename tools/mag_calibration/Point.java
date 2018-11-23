public class Point {
    float x = 0;
    float y = 0;
    int sample_count = 1;
    Point() { x = 0; y = 0; }
    Point(float _x, float _y) { x = _x; y = _y; }
    
    float getAngle() {
        return (float)Math.atan2(y , x);
    }
    float getLen() {
        return (float)Math.sqrt(x*x + y*y);   
    }
}
