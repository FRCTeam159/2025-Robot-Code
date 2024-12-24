// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

*widthimport edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;
   // PID controllers for drive and steer motors
  private final PIDController m_drivePIDController = new PIDController(4, 0, 0);
  private final PIDController m_turningPIDController = new PIDController(0.5,0.01,0);

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
 
  public static boolean m_field_oriented = false;
  public static boolean debug = false;

  int m_id;
  boolean m_inverted = false;
  String name;
  
  public SwerveModule(int driveMotorChannel,int turningMotorChannel,int id) {
    m_id = id;
    name = Drivetrain.chnlnames[m_id - 1];

    m_driveMotor = new CANSparkMax(driveMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, CANSparkLowLevel.MotorType.kBrushless);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Drivetrain.kDistPerRot); // inches to meters
    m_driveEncoder.setVelocityConversionFactor(Drivetrain.kDistPerRot / 60); // convert RPM to meters per second

    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setVelocityConversionFactor(Drivetrain.kRadiansPerRot / 60); // convert RPM to meters per second
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.setTolerance(Math.toRadians(1.0));
  }

  public double heading() {
    return m_turningEncoder.getPosition();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(heading());
  }
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), getRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), getRotation2d());
  }

  public double getDistance() {
    return m_driveEncoder.getPosition();
  }

  public double getVelocity() {
    return m_driveEncoder.getVelocity();
  }
  public double getRotations() {
    return m_driveEncoder.getPosition() / m_driveEncoder.getPositionConversionFactor();
  }

  public boolean isInverted() {
    return m_inverted;
  }

  public void setDriveInverted(boolean b) {
    m_inverted = b;
    m_driveMotor.setInverted(b);
  }

  public void reset() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  public boolean wheelReset() {
    return m_turningPIDController.atSetpoint();
  }
   // use a PID controller to set an explicit turn angle
  public void setAngle(double a, double d) {
    double r = Math.toRadians(a);
    m_turningPIDController.setSetpoint(r);
    double current = getRotation2d().getRadians(); // rotations in radians
    double turnOutput = m_turningPIDController.calculate(current, r);
    m_driveMotor.set(d);
    m_turningMotor.set(turnOutput);
  }

  public void resetWheel() {
    setAngle(0, 0);
  }
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation2d());
    double velocity = getVelocity();
    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(velocity, state.speedMetersPerSecond);
    double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    double turn_angle = getRotation2d().getRadians();

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = m_turningPIDController.calculate(turn_angle, state.angle.getRadians());
    double turnFeedforward = 0; //-m_turnFeedforward.calculate(state.angle.getRadians());

    double set_drive = driveOutput + driveFeedforward;
    double set_turn = turnOutput + turnFeedforward;

    m_driveMotor.setVoltage(set_drive);
    m_turningMotor.set(set_turn);

    if (debug) {
      String s = String.format("Drive p:%-1.3f Angle t:%-3.3f a:%-2.3f c:%-2.3f\n",
          getDistance(), Math.toDegrees(turn_angle), state.angle.getDegrees(), set_turn);
      SmartDashboard.putString(name, s);
    }
  }
  
  public void log() {
    if (!debug) {
      String s = String.format("Drive:%-1.3f m Angle:%-4.1f Rotations:%-4.2f\n",
          getDistance(), getRotation2d().getDegrees(), getRotations());
      SmartDashboard.putString(name, s);
    }
  }
  @Override
  public void periodic() {
    log();
    // This method will be called once per scheduler run
  }
}
