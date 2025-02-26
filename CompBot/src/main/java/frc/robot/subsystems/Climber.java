// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.kClimber;

import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objects.Motor;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   * 
   * @param kclimber
   */

  static double lowValue = 0;
  static double highValue = 3;// Inches
  private double m_setPoint = 0;
  static public final double kRotToIn = 0.1;
  static public final double kGearRatio = 4;
  static public final double kInchesPerRot = kRotToIn / kGearRatio;
  boolean m_raising = false;
  boolean m_lowering = false;
  boolean m_atUpperTarget = false;
  boolean m_atLowerTarget = false;
  DigitalOutput m_climbState = new DigitalOutput(3);

  SparkLimitSwitch m_upperLimit;

  private final PIDController m_PID = new PIDController(0.2, 0, 0);

  private Motor m_ClimberMotor;

  public Climber(int kclimber) {
    SmartDashboard.putString("Climber", "Inactive");
    m_ClimberMotor = new Motor(kClimber);
    m_ClimberMotor.setConfig(false, true, kInchesPerRot);
    m_ClimberMotor.setPosition(0);
    m_PID.setTolerance(0.2);
    m_ClimberMotor.setUpperLimit();
    m_ClimberMotor.setLowerLimit();
    m_climbState.set(false);
  }

  public void raise() {
    // Raises the climber claw
    System.out.println("Raising the Climber");
    setTargetHeight(highValue);
    reset();
    m_raising = true;
    m_climbState.set(false);
  }

  public boolean raising() {
    return m_raising;
  }

  public boolean lowering() {
    return m_lowering;
  }

  public void lower() {
    // Lowers the climber claw
    System.out.println("Lowering the Climber");
    setTargetHeight(lowValue);
    reset();
    m_lowering = true;
    m_climbState.set(false);
  }

  public void stop() {
    setTargetHeight(getHeight());
    reset();
  }

void setTargetHeight(double d) {
  m_setPoint = d;
}

void setHeight() {
    m_PID.setSetpoint(m_setPoint);
    double current = getHeight();
    double output = m_PID.calculate(current);

    
    m_ClimberMotor.set(output);
    String s = String.format("A:%-1.1f T:%-1.1f C:%-1.1f\n", current, m_setPoint, output);
    SmartDashboard.putString("Climber", s);
    System.out.println(s);
  }

  public double getHeight() {
    double height = m_ClimberMotor.getPosition();
    return height;
  }

  public boolean atUpperTarget() {
    return m_PID.atSetpoint();// || m_ClimberMotor.atUpperLimit();
  }

  public boolean atLowerTarget() {
    return m_PID.atSetpoint();// || m_ClimberMotor.atLowerLimit();
  }

  public void reset() {
    m_atUpperTarget = false;
    m_atLowerTarget = false;
    m_raising = false;
    m_lowering = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setHeight();
    if (m_raising) {
      if (m_ClimberMotor.atUpperLimit())
        stop();
      else if (atUpperTarget()) {
      m_atUpperTarget = true;
      m_raising = false;
      m_climbState.set(true);
    }
    }
    if (m_lowering) {
      if (m_ClimberMotor.atLowerLimit())
        stop();
      else if (atLowerTarget()) {
      m_atLowerTarget = true;
      m_lowering = false;
      }
    }
    System.out.println ("Climber Motor Intitialized");
    SmartDashboard.putBoolean("UpperTarget", m_atUpperTarget);
    SmartDashboard.putBoolean("LowerTarget", m_atLowerTarget);
  }
}
