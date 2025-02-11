// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberControl extends Command {
  /**
   * Creates a new ClimberControl.
   * 
   * @param m_controller
   * @param m_Climber
   */
  Climber m_climber;
  XboxController m_controller;

  public ClimberControl(Climber climber, XboxController controller) {
    m_climber = climber;
    m_controller = controller;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Climber Control int");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Climber Commands Executing");
    if (m_controller.getStartButtonPressed()) {
      if (m_climber.raising())
        m_climber.stop();
      else
    m_climber.raise();
    } else if (m_controller.getBackButtonPressed()) {
      if (m_climber.lowering())
        m_climber.stop();
      else
    m_climber.lower();
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
