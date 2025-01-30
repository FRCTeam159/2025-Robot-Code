// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import objects.Motor;

public class Arm extends SubsystemBase {

double last_heading = 0;

//static AHRS m_NAVXgyro=new AHRS(SerialPort.Port.kUSB);

private Motor m_motor=null;
private Motor m_rollermotor=null;

  /** Creates a new Arm. 
   * @param krollers */
  public Arm(int id, int krollers) {
    //m_motor=new Motor(id);
    //m_rollermotor = new Motor(krollers);
  }
  public void hold(){
    System.out.println("have coral");
  }
  public void eject(){
    System.out.println("outputting coral");
  }
  public void intake(){
    System.out.println("picking up coral");
  }
  public void decrament(){
    System.out.println("incrament arm");
  }
  public void increment(){
    System.out.println("incrament arm");
  }
  public void goToShelf(){
    System.out.println("going to shelf");
  }
  public void goToGround(){
    System.out.println("going to ground");
  }
  private void goToPos(double p) {
      System.out.println("going to " + p);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public double getAndle (){
  //       //return m_navx_offset-m_NAVXgyro.getRoll(); // returned values are negative
  //        double angle=m_NAVXgyro.getRoll()+m_navx_offset; // returned values are negative
  //        angle = unwrap(last_heading, angle);
  //        last_heading = angle;
  //        return angle;
  // }

  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }
}
