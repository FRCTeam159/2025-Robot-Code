// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(4, 0, 0);

  private final PIDController m_turningPIDController = new PIDController(
      0.5,
      0.01,
      0
      )

  /** Creates a new SwerveModule. */
  public SwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int id) {
      
    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkLowLevel.MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Drivetrain.kDistPerRot); // inches to meters
    m_driveEncoder.setVelocityConversionFactor(Drivetrain.kDistPerRot / 60); // convert RPM to meters per second

    // Set the distance (in this case, angle) in radians per pulse for the turning
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setPositionConversionFactor(Drivetrain.kRadiansPerRot); // inches to meters
    m_turningEncoder.setVelocityConversionFactor(Drivetrain.kRadiansPerRot / 60); // convert RPM to meters per second
    
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(Math.toRadians(1.0));
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
