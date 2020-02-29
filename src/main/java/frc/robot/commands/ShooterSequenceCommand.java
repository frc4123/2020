/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.IndexWheelCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterSequenceCommand extends ParallelCommandGroup{
  WaitCommand waitCommand = new WaitCommand(.5);
  /**
   * Creates a new IndexToShooter.
 * @param shooterSubsystem
 * @param hopperSubsystem
   */
//, is sequential ; is parallel
// auto aim, spin up, index;shooter, index;shooter
// IndexWheelCommand indexWheelCommand = new IndexWheelCommand(hopperSubsystem);
// IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);

  public ShooterSequenceCommand(DriveSubsystem driveSubsystem, IndexSubsystem hopperSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
     //we want to be straight when we are  new AutoAngleCommand(driveSubsystem),
      
      new ShooterCommand(shooterSubsystem).withTimeout(0.5),
  
      new IndexWheelCommand(hopperSubsystem)
    );
  }
}
