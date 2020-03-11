package frc.robot.trajectories;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

// import org.junit.*;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryTracking {
    public Trajectory blueAllyTrenchParallelBack;
    public Trajectory redAllyTrenchParallelBack, redOpponentTrenchSteal, redOpponentTrenchBack;
    public Trajectory testingTrack;
    public Trajectory trajectory2;
    private DriveSubsystem driveSubsystem;

    public TrajectoryTracking(DriveSubsystem driveSubsystem)  {


        this.driveSubsystem = driveSubsystem;
        // var path = new ArrayList<Translation2d>();
        // path.add(new Translation2d(1,0));
        TrajectoryConfig configForward = new TrajectoryConfig(DriveConstants.MAX_METERS_PER_SECOND,
            DriveConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.DRIVE_KINEMATICS);
            // .addConstraint(Constants.MiscConstants.autoVoltageConstraint);

        // TrajectoryConfig configReverse = new TrajectoryConfig(DriveConstants.MAX_METERS_PER_SECOND,
        //     DriveConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        //     // Add kinematics to ensure max speed is actually obeyed
        //     .setKinematics(DriveConstants.DRIVE_KINEMATICS)
        //     .addConstraint(Constants.MiscConstants.autoVoltageConstraint);
        //     configReverse.setReversed(true);

        double divisor = 1.0;
        testingTrack = 
                            // TrajectoryGenerator.generateTrajectory(
                            //     new Pose2d(0, 0, new Rotation2d())
                            //     ,
                            //     path
                            //     ,
                            //     new Pose2d(2, 0, new Rotation2d(0))
                            //     ,
                            //     configForward
                            // );
                            TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(0 / divisor, 0 / divisor, new Rotation2d(0)),
                                        new Pose2d(1 / divisor, 0 / divisor, new Rotation2d(Math.toRadians(135)))),
                                configForward);
        blueAllyTrenchParallelBack =TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(12.897 / divisor, -0.69215 / divisor, new Rotation2d(0)),
                                        new Pose2d(9.897 / divisor, -0.69215 / divisor, new Rotation2d(0))),
                                configForward);
        // blueOpponentTrenchSteal =
        // blueOpponentTrenchBack = 
        redAllyTrenchParallelBack =TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(3.048 / divisor, -2.404364 / divisor, new Rotation2d(0)),
                                        new Pose2d(6.048 / divisor, -2.404364 / divisor, new Rotation2d(0))),
                                configForward);
        redOpponentTrenchSteal =   TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(3.048 / divisor, 7.831836 / divisor, new Rotation2d(0)),
                                    new Pose2d(5.249926 / divisor, 7.831836 / divisor, new Rotation2d(0))),
                                configForward);

        redOpponentTrenchBack =   TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(5.249926 / divisor, 7.831836 / divisor, new Rotation2d(0)),
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
