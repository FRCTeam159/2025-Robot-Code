// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.kClimber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import objects.Motor;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. 
   * @param kclimber */

static double lowValue=0;
static double highValue=6;//Inches
private double m_setPoint=0;


  private final PIDController m_PID = new PIDController(0.004, 0, 0);

  private Motor m_ClimberMotor;
  public Climber(int kclimber) {
    m_ClimberMotor = new Motor (kClimber);
    m_PID.setTolerance(3);
    
  }
  public void raise(){
    //Raises the climber claw
    System.out.println("Raising the Climber");
  }
  public void lower(){
    //Lowers the climber claw
    System.out.println("Lowering the Climber");
  }

void setTargetHeight(double d) {
  m_setPoint = d;
}

void setHeight() {
    m_PID.setSetpoint(m_setPoint);
    double current = getHeight();
    double output = m_PID.calculate(current);
    
    m_ClimberMotor.set(output);
    //String s=String.format("A:%-1.1f T:%-1.1f C:%-1.1f\n", current, armSetAngle, output);
    //SmartDashboard.putString("Arm", s);
    //System.out.println(s);  
  }

  public double getHeight() {
    double height = m_ClimberMotor.getPosition();
    return height;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
