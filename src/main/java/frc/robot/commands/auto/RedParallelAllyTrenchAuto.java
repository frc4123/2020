/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.trajectories.TrajectoryTracking;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RedParallelAllyTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new TestPath.
   * 
   */
  public RedParallelAllyTrenchAuto(TrajectoryTracking trajectoryPath, ShooterSubsystem shooterSubsystem, IndexSubsystem indexSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(trajectoryPath.getRamsete(trajectoryPath.redAllyTrenchParallelBack)
          .andThen(new WaitCommand(.2).andThen(new ShooterCommand(shooterSubsystem))
          .alongWith(new WaitCommand(1).andThen(new IndexerCommand(indexSubsystem))).withTimeout(7))
    );
  }
}
