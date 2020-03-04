/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngleCommand extends PIDCommand {
  /**
   * Creates a new PIDCommandDebug.
   */

  DriveSubsystem driveSubsystem;
  double theParticularChangeInAnglularPositionWeWouldLike;
  static double setterpoint;

  public TurnToAngleCommand(DriveSubsystem driveSubsystem) {

    super(
        // The controller that the command will use
        new PIDController(1.5, 0.1, .1),
        // This should return the measurement
        () -> driveSubsystem.getGyroAngle(),
        // This should return the setpoint (can also be a constant)
        () -> setterpoint,
        // This uses the output
        output -> {
          output += Math.signum(output) * 13;
          // Use the output here
          driveSubsystem.setOutputVoltage(output, -output);
        }, driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(AutoAimConstants.ANGLE_TOLERANCE);
    this.driveSubsystem = driveSubsystem;
    // System.out.println("setterpoint: " + setterpoint);

  }

  // Returns true when the command should end.
  @Override
  public void initialize() {

    driveSubsystem.setVoltageCompensation(true, MiscConstants.TURN_VOLTAGE_COMPENSATION_VOLTS);
    // Enable this ^ for auto aim
    setterpoint = driveSubsystem.getGyroAngle() - theParticularChangeInAnglularPositionWeWouldLike;

    super.initialize();
    // System.out.println("Initialized pidcommand");

  }

  public void setThePerticularChangeInAnglularPositionWeWouldLike(
      double thePerticularChangeInAnglularPositionWeWouldLike) {
    this.theParticularChangeInAnglularPositionWeWouldLike = thePerticularChangeInAnglularPositionWeWouldLike;
  }

  @Override
  public void execute() {
    // System.out.println("turntoangle_position_error " + getController().getPositionError());
    super.execute();
  }

  @Override
  public void schedule(boolean interruptible) {
    super.schedule(interruptible);
    // System.out.println("Schedule");
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    // System.out.println("end");
    driveSubsystem.setVoltageCompensation(false, MiscConstants.TURN_VOLTAGE_COMPENSATION_VOLTS);
  }

  @Override
  public boolean isFinished() {

    return getController().atSetpoint();
  }
}
