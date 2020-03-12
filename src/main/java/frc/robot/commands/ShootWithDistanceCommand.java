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

  int topMotorSpeed;
  int bottomMotorSpeed;

  TopShooterPIDCommand topshooterPID;

  public ShootWithDistanceCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    topshooterPID = new TopShooterPIDCommand(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
    target3D = table.getEntry("tablePose");
  }

  private void getDistance() {
    double[] defaultArray = { 0.0, 0.0, 0.0 };
    double targetDistance = target3D.getDoubleArray(defaultArray)[0];

    int defaultEncoderSpeedTop = 0;
    int defaultEncoderSpeedBottom = 0;


    boolean threeMeters = (targetDistance <= 3);
    boolean threeToFourMeters = (targetDistance > 3 && targetDistance <= 4);
    boolean fourToSixMeters = (targetDistance > 4 && targetDistance <= 6);

    if (threeMeters) {
      topMotorSpeed = 100;
      bottomMotorSpeed = 100;
    } else if (threeToFourMeters) {
      topMotorSpeed = 200;
      bottomMotorSpeed = 200;
    } else if (fourToSixMeters) {
      topMotorSpeed = 300;
      bottomMotorSpeed = 300;
    } else if (fourToSixMeters) {
      topMotorSpeed = 400;
      bottomMotorSpeed = 400;
    } else {
      topMotorSpeed = defaultEncoderSpeedTop;
      bottomMotorSpeed = defaultEncoderSpeedBottom;
    }
    shootInPID();
  }

private void shootInPID() {
  int encoderChange = shooterSubsystem.getTopEncoderVelocity() - topMotorSpeed;
  topshooterPID.setPIDError(encoderChange);
  topshooterPID.schedule();
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    topshooterPID.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return topshooterPID.getController().atSetpoint();
  }
}