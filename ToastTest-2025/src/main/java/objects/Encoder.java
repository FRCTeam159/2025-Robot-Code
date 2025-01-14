// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package objects;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;


/** Add your docs here. */
public class Encoder {
     CANcoder can_encoder;
    double offset=0;
    CANcoderConfiguration config = new CANcoderConfiguration();

   
    public Encoder(int id) {
<<<<<<< HEAD
         can_encoder = new CANcoder(id);
        setOffset(0);
=======
        can_encoder = new CANcoder(id);
         config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // -0.5 to 0.5 in revolutions (Signed_PlusMinusHalf) 
        can_encoder.getConfigurator().apply(config);
        //can_encoder.setPosition(0);
        offset=0;
 
        //setOffset(0);
>>>>>>> d44114f096bb0a573c1570a569f2b85943c9d543
    }
    public void setOffset(double d){
        offset=d;
<<<<<<< HEAD
        //config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; 
=======
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // -0.5 to 0.5 in revolutions (Signed_PlusMinusHalf) 
>>>>>>> d44114f096bb0a573c1570a569f2b85943c9d543
        config.MagnetSensor.MagnetOffset = -offset; // in rotations   
        can_encoder.getConfigurator().apply(config);
        //can_encoder.setPosition(0);
        System.out.println("Offset = " + offset);
    }
   
    public double getPosition() {// return rotations
        StatusSignal<Angle> val = can_encoder.getPosition(true);
        double pos =val.getValueAsDouble();
        double gpos = pos;//-offset;
        //System.out.println("pos = " + pos + " getpos = " + gpos);
        return gpos;
    }
<<<<<<< HEAD
=======
    public double getAbsolutePosition() {// return rotations
        StatusSignal<Angle> val = can_encoder.getAbsolutePosition(true);
        return val.getValueAsDouble()-offset; 
    }
>>>>>>> d44114f096bb0a573c1570a569f2b85943c9d543
    public AngularVelocity getRate() { // return rotations/s
        StatusSignal<AngularVelocity> val = can_encoder.getVelocity();
        return val.getValue(); // return radians for rotations
     }

}
