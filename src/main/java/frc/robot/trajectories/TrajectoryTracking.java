package frc.robot.trajectories;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

// import org.junit.*;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryTracking {
    public Trajectory allyTrenchBackParallel;
    public Trajectory opponentTrenchSteal, opponentTrenchBack;
    public Trajectory centerAutoForwardTurn;
    public Trajectory trajectory2;
    private DriveSubsystem driveSubsystem;

    public TrajectoryTracking(DriveSubsystem driveSubsystem)  {
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

        double divisor = 1.0;
        centerAutoForwardTurn = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(0 / divisor, 0 / divisor, new Rotation2d(0)),
                                        new Pose2d(2 / divisor, 0 / divisor, new Rotation2d(0))),
                                configForward);
        allyTrenchBackParallel =TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(3.048 / divisor, 2.404364 / divisor, new Rotation2d(Math.PI)),
                                        new Pose2d(3.7 / divisor, 2.404364 / divisor, new Rotation2d(Math.PI))),
                                configReverse);
        opponentTrenchSteal =   TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(3.048 / divisor, 7.831836 / divisor, new Rotation2d(Math.PI)),
                                    new Pose2d(5.249926 / divisor, 7.831836 / divisor, new Rotation2d(Math.PI))),
                                configReverse);
        opponentTrenchBack =   TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(5.249926 / divisor, 7.831836 / divisor, new Rotation2d(Math.PI)),
                                        new Pose2d(3.81 / divisor, 4.125214 / divisor, new Rotation2d(-Math.toRadians(24.31)))),
                                configForward);
        try {
            trajectory2 = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/StraightPath.wpilib.json"));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    public RamseteCommand getRamsete(Trajectory traj) {
        return new RamseteCommand(traj, driveSubsystem::getPose,
                        new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
                        new SimpleMotorFeedforward(PIDConstants.KS_FEEDFOWARD, PIDConstants.KV_FEEDFOWARD, PIDConstants.KA_FEEDFOWARD),
                        DriveConstants.DRIVE_KINEMATICS, driveSubsystem::getWheelSpeeds,
                        new PIDController(PIDConstants.OPTIMAL_KP, 0, 0),
                        new PIDController(PIDConstants.OPTIMAL_KP, 0, 0), driveSubsystem::setOutputVoltage, driveSubsystem);
}
// @Test

}
