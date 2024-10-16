// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  static public final double kTurnGearRatio = 30; // MK4i turn (all)

  public static final double kWheelRadius = 2;

public static final double kDistPerRot = (Units.inchesToMeters(kWheelRadius) * 2 * Math.PI) / kDriveGearRatio;
public static final double kRadiansPerRot = Math.PI * 2 / kTurnGearRatio;

/** Creates a new Drivetrain. */
  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void init() {
  }
}
