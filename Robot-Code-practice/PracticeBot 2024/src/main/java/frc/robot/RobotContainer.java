// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.commands.DriveWithGamePad;

public class RobotContainer {

  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final DriveWithGamePad m_DriveWithGamepad = new DriveWithGamePad(m_drivetrain, m_controller);


  public void robotInit(){
    m_drivetrain.init();
    //m_detector
  }
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
