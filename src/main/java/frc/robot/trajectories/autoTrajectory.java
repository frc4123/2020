package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class autoTrajectory {
    public Trajectory allyTrenchBack;

    private DriveSubsystem driveSubsystem;

    public autoTrajectory(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), DriveConstants.DRIVE_KINEMATICS, 0);
    }
    // public RamseteCommand getRamsete(Trajectory trajectory) {
    //     return new RamseteComand(trajectory, driveSubsystem::getPose,
    //            new )
    }
}