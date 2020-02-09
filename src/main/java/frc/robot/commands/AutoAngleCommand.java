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
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.subsystems.DriveSubsystem;


public class AutoAngleCommand extends CommandBase {
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

  TurnToAngleCommand turnToAngle;

  public AutoAngleCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsytem = driveSubsystem;
    addRequirements(driveSubsystem);

    turnToAngle = new TurnToAngleCommand(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");

    targetX = table.getEntry("targetYaw");
    targetY = table.getEntry("targetPitch");

    turnBasedOnCamera();
    
  }
  
  private void turnBasedOnCamera(){
    double cameraSetpoint = targetX.getDouble(0.0) * 0.7;

    turnToAngle.setThePerticularChangeInAnglularPositionWeWouldLike(-cameraSetpoint);
    CommandGroupBase.clearGroupedCommand(turnToAngle);
    System.out.println("turnToAngle.schedule();");
    // turnToAngle.andThen(() -> {
    //   System.out.println("finished");
    //  }).schedule();
     
     
     turnToAngle.schedule();



     


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnToAngle.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
