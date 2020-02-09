/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoAimConstants;
// import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.htm
public class TurnToAngleCommand extends PIDCommand {
  /**
   * Creates a new TurnToAngle.
   */
   static double setpoint;
   DriveSubsystem driveSubsystem;
   double thePerticularChangeInAnglularPositionWeWouldLike=0;

   public TurnToAngleCommand(DriveSubsystem driveSubsystem) {

   //setpoint = driveSubsystem.getGyroAngle() - AutoAimConstants.ANGLE_SETPOINT;


    super(
        // The controller that the command will use
        new PIDController(.05, 0.006, .01),
        // This should return the measurement
        () -> driveSubsystem.getGyroAngle() % 360,
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
         // Use the output here
         driveSubsystem.setOutput(output, -output);
         System.out.println("OUTPUT " + output);
        }, driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
     
        getController().enableContinuousInput(0, 360);
    getController().setTolerance(AutoAimConstants.ANGLE_TOLERANCE);
    
    

      this.driveSubsystem = driveSubsystem;


        
      SmartDashboard.putNumber("angle_controller_kp", 2.000000);
      SmartDashboard.putNumber("angle_controller_ki", 0.001000);
      SmartDashboard.putNumber("angle_controller_kd", 0.050000);

  }
    // Configure additional PID options by calling `getController` here.

  @Override
  public void initialize() {
    
    super.initialize();
    driveSubsystem.setVoltageCompensation(true, Constants.TURN_VOLTAGE_COMPENSATION_VOLTS);
    setpoint = driveSubsystem.getGyroAngle() - thePerticularChangeInAnglularPositionWeWouldLike;

    
 }

 /**
  * @param thePerticularChangeInAnglularPositionWeWouldLike the thePerticularChangeInAnglularPositionWeWouldLike to set
  */
 public void setThePerticularChangeInAnglularPositionWeWouldLike(
     double thePerticularChangeInAnglularPositionWeWouldLike) {
   this.thePerticularChangeInAnglularPositionWeWouldLike = thePerticularChangeInAnglularPositionWeWouldLike;
 }


 @Override
 public void execute() {
   super.execute();
   SmartDashboard.putNumber("turntoangle_position_error", getController().getPositionError());
 }

  
   
 @Override
 public void schedule(boolean interruptible) {
   
   super.schedule(interruptible);

   getController().setP(SmartDashboard.getNumber("angle_controller_kp", 0));
   getController().setI(SmartDashboard.getNumber("angle_controller_ki", 0));
   getController().setD(SmartDashboard.getNumber("angle_controller_kd", 0));
 }

 @Override
 public void end(boolean interrupted) {
   super.end(interrupted);
   driveSubsystem.setVoltageCompensation(false, Constants.TURN_VOLTAGE_COMPENSATION_VOLTS);
 }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("isFinished: " +  getController().atSetpoint());
    return  getController().atSetpoint();
    

  }
}
