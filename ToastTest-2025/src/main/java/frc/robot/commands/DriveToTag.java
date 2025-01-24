// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;

public class DriveToTag extends Command {
  /** Creates a new DriveToTag. */
  Drivetrain m_drive;
  PIDController m_PID;
  double m_target=0.5;
  boolean m_started=false;

  public DriveToTag(Drivetrain drive) {
    m_PID = new PIDController(0.1, 0, 0);
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Drive to tag");
    m_started=false;
    m_PID.setSetpoint(m_target);
    m_PID.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(TagDetector.haveTag()){
      double s = TagDetector.tagDistance();
      double d = m_PID.calculate(s, m_target);
      System.out.println("distance = " + s + " correction = " + d);
      m_drive.drive(d, 0, 0, false);
      m_started=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveToTag.end " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_started && !TagDetector.haveTag()) {
      System.out.println("Lost Tags");
      return true;
    }
    boolean atTarget = m_PID.atSetpoint();
    if(atTarget)
      System.out.println("DriveToTag Target Reached");
    return atTarget;
  }
}
