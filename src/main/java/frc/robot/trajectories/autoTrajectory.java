package frc.robot.trajectories;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

public class autoTrajectory {
    public Trajectory allyTrenchBack;
    public Trajectory trajectory2;
    private DriveSubsystem driveSubsystem;

    public autoTrajectory(DriveSubsystem driveSubsystem) throws IOException {
        this.driveSubsystem = driveSubsystem;

        TrajectoryConfig configForward = new TrajectoryConfig(DriveConstants.MAX_METERS_PER_SECOND,
            DriveConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS)
            .addConstraint(Constants.MiscConstants.autoVoltageConstraint);

        TrajectoryConfig configReverse = new TrajectoryConfig(DriveConstants.MAX_METERS_PER_SECOND,
            DriveConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.DRIVE_KINEMATICS)
        .addConstraint(Constants.MiscConstants.autoVoltageConstraint);
        configReverse.setReversed(true);

        trajectory2 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/StraightPath.wpilib.json"));
    }
    public RamseteCommand getRamsete(Trajectory traj) {
        return new RamseteCommand(traj, driveSubsystem::getPose,
                        new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
                        new SimpleMotorFeedforward(PIDConstants.KS_FEEDFOWARD, PIDConstants.KV_FEEDFOWARD, PIDConstants.KA_FEEDFOWARD),
                        DriveConstants.DRIVE_KINEMATICS, driveSubsystem::getWheelSpeeds,
                        new PIDController(PIDConstants.OPTIMAL_KP, 0, 0),
                        new PIDController(PIDConstants.OPTIMAL_KP, 0, 0), driveSubsystem::setOutputVoltage, driveSubsystem);
}
}