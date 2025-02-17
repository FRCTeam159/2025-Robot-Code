// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objects.Motor;

public class Arm extends SubsystemBase {

  double last_heading = 0;
  static double m_navx_offset = 0;// 83.1; // observed gyro value when arm is horizontal
  static double shelfAngle = 180;
  static double groundAngle = 190;
  static double testAngle = 90;
  static boolean use_trap_pid=true;

  static public final double kGearRatio = 80*12.0/14.0;
  public static final double kDegreesPerRot = 360 / (kGearRatio);
  // shelf pos is 132
  // floor pos is 200

  private PIDController m_PID;
  private ProfiledPIDController m_tPID;
  static AHRS m_NAVXgyro = new AHRS(NavXComType.kUSB1);

  private Motor m_armPosMotor = null;
  private Motor m_topRollerMotor = null;
  private Motor m_bottomRollerMotor = null;

  static final double MAX_ANGLE = 200;
  static final double MIN_ANGLE = 0;
  boolean m_intake = false;
  boolean m_eject = false;
  double intakeValue = 2;
  double ejectValue = -2;

  DigitalInput m_coralSensor = new DigitalInput(1);
  DigitalOutput m_coralState = new DigitalOutput(2);

  boolean newAngle = true;
  private double armSetAngle = 0;

  /**
   * Creates a new Arm.
   * 
   * @param krollers
   */
  public Arm(int armId, int bottomRollers, int topRollers) {
    if(use_trap_pid){
      m_tPID=new ProfiledPIDController(0.01, 0, 0,
        new TrapezoidProfile.Constraints(100,100));
      m_tPID.setTolerance(1);
      m_tPID.reset(0);
    }
    else{
      m_PID = new PIDController(0.005, 0, 0);
      m_PID.setTolerance(1);
      m_PID.reset();
    }
    // SmartDashboard.putNumber("NavX", 0);
    SmartDashboard.putString("Arm", "Inactive");
    if ((Constants.testMode == Constants.test.ONEROLLER) || (Constants.testMode == Constants.test.TWOROLLERS)) {
      m_topRollerMotor = new Motor(topRollers, false);
      m_topRollerMotor.setConfig(false, 1);
      m_topRollerMotor.setPosition(0);
      m_topRollerMotor.enable();
      if (Constants.testMode == Constants.test.TWOROLLERS) {
        m_bottomRollerMotor = new Motor(bottomRollers, false);
        m_bottomRollerMotor.setConfig(false, 1);
        m_bottomRollerMotor.setPosition(0);
        m_bottomRollerMotor.enable();
      }
    } else {
      if (Constants.testMode == Constants.test.ARMGYRO){
        m_armPosMotor = new Motor(armId, true);
        m_armPosMotor.setConfig(false, kDegreesPerRot);
      }
      else{
        m_armPosMotor = new Motor(armId, false);
        m_armPosMotor.setConfig(true, kDegreesPerRot);
      }
      m_armPosMotor.setPosition(0);
      // m_rollermotor = new Motor(krollers);
      m_armPosMotor.enable();
    }
  }

  public boolean coralAtIntake() {
    // return !noteSensor1.get();
    return m_coralSensor.get();
  }

  public void adjustAngle(double adjustment) {
    setNewTarget(armSetAngle + adjustment);
  }

  void setNewTarget(double angle) {
    angle=angle>MAX_ANGLE?MAX_ANGLE:angle;
    angle=angle<MIN_ANGLE?MIN_ANGLE:angle;
    armSetAngle = angle;
    setPID(angle);
  }

  void setPID(double a){
    if(use_trap_pid)
      m_tPID.setGoal(a);
    else
      m_PID.setSetpoint(a);
  }
  double getPID(double c){
    if(use_trap_pid)
      return m_tPID.calculate(c);
    else
      return m_PID.calculate(c);
  }
  void setAngle() {
    double current = getAngle();
    double output = getPID(current);
    m_armPosMotor.set(output);
    String s = String.format("A:%-1.1f T:%-1.1f C:%-1.1f\n", current, armSetAngle, output);
    SmartDashboard.putString("Arm", s);
    // System.out.println(s);
  }

  public void hold() {
    System.out.println("have coral");
  }

  public void eject() {
    m_eject = true;
    System.out.println("outputting coral");
  }

  public void intake() {
    m_intake = true;
    System.out.println("picking up coral");
  }

  public boolean rollersOn() {
    return m_intake || m_eject;
  }

  public void stopRollers() {
    m_intake = false;
    m_eject = false;
  }

  public void setRollers() {
    double rollerSpeed;
    if (m_intake)
      rollerSpeed = intakeValue;
    else if (m_eject)
      rollerSpeed = ejectValue;
    else
      rollerSpeed = 0;
    m_topRollerMotor.setVoltage(rollerSpeed);
    if (Constants.testMode == Constants.test.TWOROLLERS) 
      m_bottomRollerMotor.setVoltage(-rollerSpeed);
    String s = String.format("Eject:%b Intake:%b Speed:%1.2f", m_eject, m_intake, rollerSpeed);
    SmartDashboard.putString("Arm", s);
  }

  public void decrement(double angle) {
    // System.out.println("decrament arm by " + angle);
    adjustAngle(-angle);
  }

  public void increment(double angle) {
    // System.out.println("incrament arm by " + angle);
    adjustAngle(angle);
  }

  public void goToShelf() {
    System.out.println("going to shelf");
    setNewTarget(shelfAngle);
  }

  public void goToTest() {
    System.out.println("going to test");
    setNewTarget(testAngle);
  }

  public void goToGround() {
    System.out.println("going to ground");
    setNewTarget(groundAngle);
  }

  public void goToZero() {
    System.out.println("going to zero");
    setNewTarget(0);
  }

  @Override
  public void periodic() {
    //
    SmartDashboard.putBoolean("CoralDetected", coralAtIntake());
    m_coralState.set(coralAtIntake());
    if (Constants.testMode == Constants.test.ONEROLLER || Constants.testMode == Constants.test.TWOROLLERS)
      setRollers();
    else
      setAngle();
  }

  public double getAngle() {
    double angle = 0;
    if (Constants.testMode == Constants.test.ARMGYRO)
      angle = m_NAVXgyro.getRoll() + m_navx_offset; // returned values are negative
    else
      angle = m_armPosMotor.getPosition();
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
