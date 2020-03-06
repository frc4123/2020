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

  double[] defaultArray;
  double targetDistance;
  double topVoltage;
  double botVoltage;

  public ShootWithDistanceCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    this.shooterSubsystem = shooterSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] defaultArray = { 0.0, 0.0, 0.0 };
    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
    target3D = table.getEntry("targetPose");
    targetDistance = target3D.getDoubleArray(defaultArray)[0];
    getDistance();
  }

  private void getDistance() {

    // boolean threeMeters = (targetDistance <= 3);
    boolean threeToSixMeters = (targetDistance > 3 && targetDistance <= 6);
    boolean sixToEight = (targetDistance > 6 && targetDistance <= 8);
    // if (threeMeters) {
    // topVoltage = 11;
    // botVoltage = 12;
    // }
    if (threeToSixMeters) {
      topVoltage = 11.5;
      botVoltage = 12;
    } else if (sixToEight) {
      topVoltage = 12;
      botVoltage = 12;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(targetDistance);
    shooterSubsystem.setTopShooterMotorVoltage(topVoltage);
    shooterSubsystem.setBottomShooterMotorVoltage(botVoltage);
    System.out.println(topVoltage);
    System.out.println(botVoltage);
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