// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import objects.Motor;

public class Climber extends SubsystemBase {

private Motor m_motor=null;

  /** Creates a new Climber. */
  public Climber(int id) {
    m_motor=new Motor(id);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
