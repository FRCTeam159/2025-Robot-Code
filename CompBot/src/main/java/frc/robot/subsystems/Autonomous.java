package frc.robot.subsystems;

import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.DriveStraight;

public class Autonomous {
    Drivetrain m_drivetrain;
    TagDetector m_Detector;
    public static final int DRIVE_STRAIGHT = 1; 
    public static final int DRIVE_TO_TAG = 2;
    static SendableChooser<Integer> m_autochooser = new SendableChooser<Integer>();

    public Autonomous(Drivetrain drivetrain, TagDetector Detector) {
        m_drivetrain = drivetrain;
        m_Detector = Detector;
        m_autochooser.setDefaultOption("Drive Straight", DRIVE_STRAIGHT);
        m_autochooser.addOption("Drive To Tag", DRIVE_TO_TAG);
        SmartDashboard.putData(m_autochooser);
    }

    public SequentialCommandGroup getCommand() {
        int automode = m_autochooser.getSelected();
        switch (automode) {
            case DRIVE_STRAIGHT:
                System.out.println("Drive straight");
                return new SequentialCommandGroup(new DriveStraight(m_drivetrain));
            case DRIVE_TO_TAG:
                System.out.println("Drive to tag");
                return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
        }
        
        //driveToTag();
        return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
    }
}
