// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package objects;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class Motor {
    CANSparkMax rev_motor=null;
    RelativeEncoder rev_encoder;
    int m_chnl;
    boolean m_inverted=false;
    boolean m_enabled=true;

    double m_dpr=1;
    static boolean m_real=false;
     public Motor(int id) {
        rev_motor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
        rev_encoder = rev_motor.getEncoder();
         m_chnl=id;
    }
    static public void setMode(boolean m){
        m_real=m;
    }
    public void enable(){
        m_enabled=true;
      }
    public void disable(){
        m_enabled=false;
        rev_motor.disable();
    }
    public double getPosition() {
        return rev_encoder.getPosition();
     }
    public void setPosition(double d) {
            rev_encoder.setPosition(d);
    }
     public double getRotations(){
        return getPosition()/m_dpr;
    }
     public void reset(){
        rev_encoder.setPosition(0.0);
     }
    public double getVelocity() {
        return rev_encoder.getVelocity()/60; // rpm to rps
    }
    public void setInverted() {
        rev_motor.setInverted(true);
    }
    public void set(double speed) {
        rev_motor.set(speed);
       }
    public void setVoltage(double v) {
        rev_motor.setVoltage(v);
        }
    public void setDistancePerRotation(double d){
        m_dpr=d;
        rev_encoder.setPositionConversionFactor(d);
    }
}
