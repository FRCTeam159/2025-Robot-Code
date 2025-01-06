package objects;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro {
     static boolean is_real=false;
        ADXRS450_Gyro real_gyro=null;

     public Gyro() {
          real_gyro=new ADXRS450_Gyro();
    }
    
    static public void setMode(boolean m){
        is_real=m;
    }

    public void reset(){
         real_gyro.reset();
    }
    public double getAngle(){
        return -real_gyro.getAngle();
    }   
}
