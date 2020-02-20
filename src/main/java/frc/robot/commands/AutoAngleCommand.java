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

public class AutoAngleCommand extends CommandBase {
  /**
   * Creates a new AutoAimCommand.
   */

  static double setpoint;

  double thePerticularChangeInAnglularPositionWeWouldLike;

  double rotationError;
  double distanceError;
  double rotationAdjust;

  NetworkTable table;
  NetworkTableEntry targetX;
  NetworkTableEntry targetY;

  PIDCommandDebug debug;
  DriveSubsystem driveSubsystem;

  public AutoAngleCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    // turnToAngle = new TurnToAngleCommand(driveSubsystem);
    debug = new PIDCommandDebug(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");

    targetX = table.getEntry("targetYaw");
    targetY = table.getEntry("targetPitch");

    turnBasedOnCamera();

    // setpoint = driveSubsystem.getGyroAngle() - 90;
    // driveSubsystem.setVoltageCompensation(true,
    // Constants.TURN_VOLTAGE_COMPENSATION_VOLTS);
    // super.initialize();
    setpoint = driveSubsystem.getGyroAngle() - thePerticularChangeInAnglularPositionWeWouldLike;

  }

  private void turnBasedOnCamera() {
    double cameraSetpoint = targetX.getDouble(0.0) - (.2 * targetX.getDouble(0.0));
    // 80 ppercent is pretty darn close offset
    // offset is betwee 75 - 85

    // maybe subtract an offset

    debug.setThePerticularChangeInAnglularPositionWeWouldLike(-cameraSetpoint);

    // CommandGroupBase.clearGroupedCommand(turnToAngle);
    // System.out.println("debug.schedule();");
    System.out.println("turnToAngle.schedule();");
    // turnToAngle.andThen(() -> {
    // System.out.println("finished");
    // }).schedule();

    debug.schedule();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    debug.cancel();
    // turnToAngle.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
