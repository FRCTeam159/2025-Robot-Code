// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package objects;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class Motor {
    //private CANSparkMax sparkMax = new CANSparkMax(deviceID, MotorType.kBrushless);
    SparkMax rev_motor=null;
    RelativeEncoder rev_encoder;
    int m_chnl;
    boolean m_inverted=false;
    boolean m_enabled=true;
    
    double m_dpr=1;
    static boolean m_real=false;
    public Motor(int id, boolean isBrushed) {
        rev_motor = new SparkMax(id, isBrushed?MotorType.kBrushed:MotorType.kBrushless);
        rev_encoder = rev_motor.getEncoder();
        m_chnl=id;
    }
    public Motor(int id) {
        rev_motor = new SparkMax(id, MotorType.kBrushless);
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
    
    public void set(double speed) {
        rev_motor.set(speed);
       }
    public void setVoltage(double v) {
        rev_motor.setVoltage(v);
    }
    public void setConfig(boolean isInverted, double d){
        m_dpr=d;
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(isInverted);
        //    .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(d)
            .velocityConversionFactor(d/60);
        //config.closedLoop
         //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //    .pid(1.0, 0.0, 0.0);
        
        rev_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
