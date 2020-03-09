/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.trajectories.TrajectoryTracking;

import frc.robot.trajectories.TrajectoryTracking;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DefaultAuto extends SequentialCommandGroup {
  /**
   * Creates a new DefaultAuto.
   */
  public DefaultAuto(TrajectoryTracking trajectoryPath) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(trajectoryPath.getRamsete(trajectoryPath.trajectory2));
  }
}
