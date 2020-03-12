/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TopShooterPIDCommand extends PIDCommand {
  /**
   * Creates a new ShooterPIDCommand.
   */
  ShooterSubsystem shooterSubsystem;
  int encoderError;

  public TopShooterPIDCommand(ShooterSubsystem shooterSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(1.00, 0.008, 0.05),
        // This should return the measurement
        () -> shooterSubsystem.getTopEncoderVelocity(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  public int setPIDError(int encoderChange) {
    return this.encoderError = encoderChange;
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
