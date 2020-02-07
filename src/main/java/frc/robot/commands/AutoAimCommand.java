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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoAimConstants;

public class AutoAimCommand extends CommandBase {
  /**
   * Creates a new AutoAimCommand.
   */

  NetworkTable table;
  NetworkTableEntry targetX;
  NetworkTableEntry targetY;

  double rotationError;
  double distanceError;
  double rotationAdjust;

  DriveSubsystem driveSubsytem;

  public AutoAimCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsytem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");

    targetX = table.getEntry("targetYaw");
    targetY = table.getEntry("targetPitch");

    // one time equaTions
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("printblabhalbahbalhbah");
    // poll tx and ty and
    rotationError = targetX.getDouble(0.0);
    distanceError = targetY.getDouble(0.0);
    // rotationAjust = rotationError;

    if (Math.abs(rotationError) > AutoAimConstants.ANGLE_TOLERANCE) {
      rotationAdjust = AutoAimConstants.KP_ROTATION_AUTOAIM * rotationError
          + rotationError / Math.abs(rotationError) * AutoAimConstants.KF_AUTOAIM;
    } else {
      rotationAdjust = 0;
    }

    // else {
    // if (rotationError < AutoAimConstants.ANGLE_TOLERANCE) {
    // rotationAdjust = AutoAimConstants.KP_ROTATION_AUTOAIM * rotationError -
    // AutoAimConstants.KF_AUTOAIM;
    // }
    // }

    driveSubsytem.arcadeDrive(0, rotationAdjust);
    System.out.println(rotationAdjust);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
