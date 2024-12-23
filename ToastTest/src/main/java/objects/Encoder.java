// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package objects;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;


/** Add your docs here. */
public class Encoder {
     CANcoder can_encoder;
    double offset=0;
   
    public Encoder(int id) {
         can_encoder = new CANcoder(id);
        setOffset(0);
    }
    public void setOffset(double d){
        CANcoderConfiguration config = new CANcoderConfiguration();
        offset=d;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; 
        config.MagnetSensor.MagnetOffset = -offset; // in rotations   
        can_encoder.getConfigurator().apply(config);
    }
   
    public double getPosition() {// return rotations
        StatusSignal<Double> val = can_encoder.getPosition();
        return val.getValue(); 
    }
    public double getRate() { // return rotations/s
        StatusSignal<Double> val = can_encoder.getVelocity();
        return val.getValue(); // return radians for rotations
     }

}
