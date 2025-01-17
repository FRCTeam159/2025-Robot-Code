// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;

public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_drivetrain, m_controller);

  private final TagDetector m_Detector= new TagDetector(m_drivetrain);

  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_DriveWithGamepad);
  }

  public void robotInit() {
    m_drivetrain.init();
    m_drivetrain.reset();
    m_Detector.start();
  }
  public void teleopInit(){
    m_drivetrain.enable();
  }
  public void disabledInit(){
    m_drivetrain.disable();
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
