/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithDistanceCommand extends CommandBase {
  /**
   * Creates a new ShootWithDistanceCommand.
   */

  ShooterSubsystem shooterSubsystem;

  NetworkTable table;

  NetworkTableEntry target3D;

  double setTargetDistance;
  
  public ShootWithDistanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
    target3D = table.getEntry("tablePose");
  }

  private void getDistance() {
    double[] defaultArray = {0.0, 0.0, 0.0};
    double targetDistance = target3D.getDoubleArray(defaultArray)[0];

    boolean threeMeters = targetDistance <= 3;
    boolean threeToEightMeters = targetDistance > 3 && targetDistance <= 8;
    boolean eightToTenMeters = targetDistance > 8 && targetDistance <= 10;


    if (threeMeters) {
      setTargetDistance = 3;
    }
    else if (threeToEightMeters) {
      setTargetDistance = 5;
    }
    else if (eightToTenMeters) {
      setTargetDistance = 10;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getDistance();
    shooterSubsystem.setTopShooterMotorVoltage(setTargetDistance);
    shooterSubsystem.setBottomShooterMotorVoltage(setTargetDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setTopShooterMotorVoltage(0);
    shooterSubsystem.setBottomShooterMotorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
