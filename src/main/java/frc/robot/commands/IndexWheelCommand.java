/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VoltageConstants;
import frc.robot.subsystems.HopperSubsystem;

public class IndexWheelCommand extends CommandBase {
  
  HopperSubsystem hopperSubsystem;
  

  /**
   * Creates a new HopperToShooterThingy.
   */
  public IndexWheelCommand(HopperSubsystem hopperSubsystem) {
    addRequirements(hopperSubsystem);
    this.hopperSubsystem = hopperSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("hopper 3 volt");
    hopperSubsystem.hopperVoltage(VoltageConstants.HOPPER_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopperSubsystem.hopperVoltage(VoltageConstants.HOPPER_STOP);
    System.out.println("hopper done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
//change the name of the command class
