// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import objects.Motor;

public class Arm extends SubsystemBase {

double last_heading = 0;
static double m_navx_offset=0;//83.1; // observed gyro value when arm is horizontal
static double shelfAngle=132;
static double groundAngle=200;
// shelf pos is 132
// floor pos is 200

private final PIDController m_PID = new PIDController(0.01, 0, 0);

static AHRS m_NAVXgyro=new AHRS(NavXComType.kUSB1);

private Motor m_armPosMotor=null;
private Motor m_rollermotor=null;

static final double MAX_ANGLE=200;
static final double MIN_ANGLE=0;

boolean newAngle = true;
private double armSetAngle = 90;

  /** Creates a new Arm. 
   * @param krollers */
  public Arm(int id, int krollers) {
    SmartDashboard.putNumber("NavX",0);
    SmartDashboard.putString("Arm", "Inactive");
    m_armPosMotor=new Motor(id, true);
    m_PID.setTolerance(3);
    m_PID.reset();
    //m_rollermotor = new Motor(krollers);
    m_armPosMotor.enable();
  }
  public void adjustAngle(double adjustment) {
    setNewTarget(armSetAngle+adjustment);
  }

  void setNewTarget(double angle){
    angle=angle>MAX_ANGLE?MAX_ANGLE:angle;
    angle=angle<MIN_ANGLE?MIN_ANGLE:angle;
    armSetAngle=angle;
  }

  void setAngle() {
    m_PID.setSetpoint(armSetAngle);
    double current = getAngle();
    double output = m_PID.calculate(current);
    
    m_armPosMotor.set(output);
    String s=String.format("A:%-1.1f T:%-1.1f C:%-1.1f\n", current, armSetAngle, output);
    SmartDashboard.putString("Arm", s);
    //System.out.println(s);  
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
  public void decrement(double angle){
    System.out.println("decrament arm by " + angle);
    adjustAngle(-angle);
  }
  public void increment(double angle){
    System.out.println("incrament arm by " + angle);
    adjustAngle(angle);
  }
  public void goToShelf(){
    System.out.println("going to shelf");
    setNewTarget(shelfAngle);
  }
  public void goToGround(){
    System.out.println("going to ground");
    setNewTarget(groundAngle);
  }
  public void goToZero(){
    System.out.println("going to zero");
    setNewTarget(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NavX",getAngle());
    setAngle();
    //System.out.println("NavX Gyro " + getAngle());
    // This method will be called once per scheduler run
  }

  public double getAngle() {
    double angle = m_NAVXgyro.getRoll() + m_navx_offset; // returned values are negative
    angle = unwrap(last_heading, angle);
    last_heading = angle;
    return angle;
  }

  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }
}
