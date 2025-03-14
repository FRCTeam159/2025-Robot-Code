// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.fasterxml.jackson.databind.deser.std.StdScalarDeserializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
// import balls
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objects.Motor;
import frc.robot.utils.Averager;

public class Arm extends SubsystemBase {

  double last_heading = 0;
  static double shelfAngle = 139;
  static double groundAngle = 200;
  static double startAngle = 80;
  static boolean useTrapPID=false;

  static public final double kGearRatio = 80*38.0/22.0;
  public static final double kDegreesPerRot = 360 / (kGearRatio);
  // shelf pos is 132
  // floor pos is 200

  private PIDController m_PID;
  private ProfiledPIDController m_tPID;
  AnalogInput potentiometerInput = new AnalogInput(1);
  private Motor m_armPosMotor = null;
  private Motor m_topRollerMotor = null;
  private Motor m_bottomRollerMotor = null;

  static final double START_ANGLE = 80;
  static final double MAX_ANGLE = 200-START_ANGLE;
  static final double MIN_ANGLE = 0;
  boolean m_intake = false;
  boolean m_eject = false;
  boolean m_ejecting = false;
  double intakeValue = 5;
  double ejectValue = -3;

  DigitalInput m_coralSensor1 = new DigitalInput(1);
  DigitalInput m_coralSensor2 = new DigitalInput(0);
  DigitalOutput m_coralState = new DigitalOutput(2);
  DigitalInput m_encoderInput = new DigitalInput(4);
  DutyCycleEncoder m_dutyCycleEncoder = new DutyCycleEncoder(m_encoderInput);

  boolean newAngle = true;
  private double armSetAngle = 0;
  double m_coralAtIntake1=0;
  double m_coralAtIntake2=0;
  Averager sensor1_averager = new Averager(5);
  Averager sensor2_averager = new Averager(5);
  boolean useAltEncoder = true;
  boolean m_sensorEnable = true;

  Timer m_timer = new Timer();

  /**
   * Creates a new Arm.
   * 
   * @param krollers
   */
  public Arm(int armId, int bottomRollers, int topRollers) {
    //m_dutyCycleEncoder.setInverted(true);
    m_timer.start();
    if(useTrapPID){
      m_tPID=new ProfiledPIDController(0.005, 0, 0,
        new TrapezoidProfile.Constraints(100,50));
      m_tPID.setTolerance(2);
      m_tPID.reset(0);
    }
    else{
      m_PID = new PIDController(0.005, 0, 0);
      m_PID.setTolerance(2);
      m_PID.reset();
    }
    // SmartDashboard.putNumber("NavX", 0);
    SmartDashboard.putString("Arm", "Inactive");
    m_topRollerMotor = new Motor(topRollers, false);
    m_topRollerMotor.setConfig(false, 1);
    m_topRollerMotor.setPosition(0);
    m_topRollerMotor.enable();

    m_bottomRollerMotor = new Motor(bottomRollers, false);
    m_bottomRollerMotor.setConfig(false, 1);
    m_bottomRollerMotor.setPosition(0);
    m_bottomRollerMotor.enable();

    m_armPosMotor = new Motor(armId, false);
    m_armPosMotor.setConfig(false, true, kDegreesPerRot);
    m_armPosMotor.setPosition(0);
     // m_rollermotor = new Motor(krollers);
    m_armPosMotor.enable();
    }

  public boolean coralAtIntake1() {
    // return !noteSensor1.get();
    double val= m_coralSensor1.get()?1:0.0;
    m_coralAtIntake1 = sensor1_averager.getAve(val);
    return m_coralAtIntake1>0.5?false:true;
  }

  public boolean coralAtIntake2() {
    // return !noteSensor1.get();
    double val2= m_coralSensor2.get()?1:0.0;
    m_coralAtIntake2 = sensor2_averager.getAve(val2);
    return m_coralAtIntake2>0.5?false:true;
  }

  public boolean coralAtIntake() {
    if (coralAtIntake1() || coralAtIntake2())
      return true;
    else  
      return false;
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
    if(useTrapPID)
      m_tPID.setGoal(a);
    else
      m_PID.setSetpoint(a);
  }
  double getPID(double c){
    if(useTrapPID)
      return m_tPID.calculate(c);
    else
      return m_PID.calculate(c);
  }

  public void enable(){
    m_armPosMotor.enable();
  }
  void setAngle() {
    double current = getAngle();
    double output = getPID(current);
    m_armPosMotor.set(output);
    String s = String.format("A:%-1.1f T:%-1.1f C:%-1.1f\n", current + START_ANGLE, armSetAngle + START_ANGLE, output);
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

  public void enableSensorStop(boolean b) {
    m_sensorEnable = b;
  }

  public void setRollers() {
    double rollerSpeed = 0;
    if (m_intake){
      if (coralAtIntake() && m_sensorEnable)
        stopRollers();
      else
        rollerSpeed = intakeValue;
    }
    else if (m_eject){
      if (!m_ejecting && coralAtIntake() && m_sensorEnable){
        m_ejecting = true;
        m_timer.reset();
      }
      if (m_ejecting){
        if (m_timer.get() >= 1){
          m_ejecting = false;
          stopRollers();
        }
        else
          rollerSpeed = ejectValue;
      }
      else 
      rollerSpeed = ejectValue;
    }
    else
      rollerSpeed = 0;
    m_topRollerMotor.setVoltage(rollerSpeed);
    m_bottomRollerMotor.setVoltage(-rollerSpeed);
    String s = String.format("Eject:%b Intake:%b Speed:%1.2f", m_eject, m_intake, rollerSpeed);
    SmartDashboard.putString("Rollers", s);
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
    setNewTarget(shelfAngle-START_ANGLE);
  }

  public void goToGround() {
    System.out.println("going to ground");
    setNewTarget(groundAngle-START_ANGLE);
  }

  public void goToStart() {
    System.out.println("going to zero");
    setNewTarget(0);
  }

  public double getBoreEncoderVal() {
    double value = m_dutyCycleEncoder.get();
    double pStart = 0.9629357490733937;
    double pGround = 0.3068258326706458;
    double m = (startAngle-groundAngle)/(pStart-pGround); //max and min of the arm 90 and 210 over the max and min of the POT to find the slope a equation
    double b = startAngle-(m * pStart);
    double voltage = value;
    double x = voltage * m + b;
    //System.out.println("RawBore: " + value);
    
    return x - START_ANGLE;
  }

  @Override
  public void periodic() {
    //
    boolean coral = coralAtIntake();
    SmartDashboard.putBoolean("CoralDetected", coral);
    SmartDashboard.putNumber("Pot Value", getPotentiometerValue() + START_ANGLE);
    SmartDashboard.putBoolean("SensorAutoStop", m_sensorEnable);
   //SmartDashboard.putNumber("BoreEncoder", getBoreEncoderVal() + START_ANGLE);
    m_coralState.set(coral);
    setRollers();
    setAngle();
  }

//3.08 start/90
//3.405 ground 
  public double getPotentiometerValue() {
    double pStart = 2.202148212;
    double pGround = 2.515868883;
    double m = (startAngle-groundAngle)/(pStart-pGround); //max and min of the arm 90 and 210 over the max and min of the POT to find the slope a equation
    double b = startAngle-(m * pStart);
    double voltage = potentiometerInput.getVoltage();
    double x = voltage * m + b;
    //System.out.println("RawPot: " + voltage);
    
    return x - START_ANGLE;
  }

  public double getAngle() {
    double angle = 0;
    if (useAltEncoder)
      angle = getPotentiometerValue(); 
    else
      angle = m_armPosMotor.getPosition();
    angle = unwrap(last_heading, angle);
    last_heading = angle;
    //System.out.println(getPotentiometerValue() + " " + m_armPosMotor.getPosition());
    return angle;
  }

  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }
}
