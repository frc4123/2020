/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends CommandBase {
  
  ElevatorSubsystem elevatorSubsystem;

  public ElevatorUpCommand(ElevatorSubsystem elevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setVoltage(3);
  }

  
  @Override
  public void end(boolean interrupted) {
  elevatorSubsystem.setVoltage(0);
 }
 @Override
 public boolean isFinished() {
   //assumes normaly closed? 
   return elevatorSubsystem.isLimitTopSwitchHit();
 }
}
