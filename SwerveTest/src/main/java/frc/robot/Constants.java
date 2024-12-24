// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static final double kFrontLeftOffset = 0;//-0.405518;
    public static final double kFrontRightOffset =  0;//0.403076;
    public static final double kBackLeftOffset =  0;//0.268555;
    public static final double kBackRightOffset =  0;//0.156730;
    // can ids for drive train
    public static final int kFl_Drive = Robot.isReal()?3:1;
    public static final int kFl_Turn = Robot.isReal()?8:2;
    public static final int kFl_Encoder = Robot.isReal()?12:2;

    public static final int kFr_Drive = Robot.isReal()?7:3;
    public static final int kFr_Turn =  Robot.isReal()?6:4;
    public static final int kFr_Encoder = Robot.isReal()?10:4;

    public static final int kBl_Drive = Robot.isReal()?2:5;
    public static final int kBl_Turn  = Robot.isReal()?4:6;
    public static final int kBl_Encoder = Robot.isReal()?9:6;

    public static final int kBr_Drive = Robot.isReal()?5:7;
    public static final int kBr_Turn = Robot.isReal()?1:8;
    public static final int kBr_Encoder = Robot.isReal()?11:8;

     public static final int kSpare = Robot.isReal()?13:9;



   
}
