package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTag;

public class Autonomous {
    Drivetrain m_drivetrain;
    TagDetector m_Detector;

    public Autonomous(Drivetrain drivetrain, TagDetector Detector) {
        m_drivetrain = drivetrain;
        m_Detector = Detector;
    }

    public SequentialCommandGroup getCommand() {
        
        //driveToTag();
        return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
    }
}
