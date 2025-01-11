// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package objects;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;


/** Add your docs here. */
public class Encoder {
     CANcoder can_encoder;
    double offset=0;
   
    public Encoder(int id) {
        can_encoder = new CANcoder(id);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // -0.5 to 0.5 in revolutions (Signed_PlusMinusHalf) 
        can_encoder.getConfigurator().apply(config);
        //can_encoder.setPosition(0);
        offset=0;
 
        //setOffset(0);
    }
    public void setOffset(double d){
        CANcoderConfiguration config = new CANcoderConfiguration();
        offset=d;
        // config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // -0.5 to 0.5 in revolutions (Signed_PlusMinusHalf) 
        config.MagnetSensor.MagnetOffset = offset; // in rotations   
        can_encoder.getConfigurator().apply(config);
        //can_encoder.setPosition(0);
    }
   
    public double getPosition() {// return rotations
        StatusSignal<Angle> val = can_encoder.getPosition(true);
        return val.getValueAsDouble();
    }
    public double getAbsolutePosition() {// return rotations
        StatusSignal<Angle> val = can_encoder.getAbsolutePosition(true);
        return val.getValueAsDouble(); 
    }
    public AngularVelocity getRate() { // return rotations/s
        StatusSignal<AngularVelocity> val = can_encoder.getVelocity();
        return val.getValue(); // return radians for rotations
     }

}
