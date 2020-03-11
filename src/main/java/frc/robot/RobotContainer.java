/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LogitecController;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.auto.RedFiveBallAuto;
import frc.robot.commands.auto.RedParallelAllyTrenchAuto;
import frc.robot.commands.auto.testTrajectory;
import frc.robot.commands.AutoAngleCommand;
import frc.robot.commands.ElevatorDropCommand;
import frc.robot.commands.ElevatorLiftCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeBallInCommand;
import frc.robot.commands.IntakeBallOutCommand;
import frc.robot.commands.IntakeDeployGateCommand;
import frc.robot.commands.IntakeRetractGateCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.WinchUpCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeGateSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.trajectories.TrajectoryTracking;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IndexSubsystem indexSubsystem = new IndexSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IntakeGateSubsystem intakeGateSubsystem = new IntakeGateSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final WinchSubsystem winchSubsystem = new WinchSubsystem();
  public final TrajectoryTracking trajectoryPath = new TrajectoryTracking(driveSubsystem);
  // Commands
  private final AutoAngleCommand autoAimCommand = new AutoAngleCommand(driveSubsystem);
  private final ElevatorLiftCommand elevatorUpCommand = new ElevatorLiftCommand(elevatorSubsystem);
  private final ElevatorDropCommand elevatorDownCommand = new ElevatorDropCommand(elevatorSubsystem);
  private final IntakeDeployGateCommand intakeGateDownCommand = new IntakeDeployGateCommand(intakeGateSubsystem);
  private final IntakeRetractGateCommand intakeGateUpCommand = new IntakeRetractGateCommand(intakeGateSubsystem);
  private final IntakeBallInCommand intakeInCommand = new IntakeBallInCommand(intakeSubsystem);
  private final IntakeBallOutCommand intakeOutCommand = new IntakeBallOutCommand(intakeSubsystem);
  private final WinchUpCommand winchUpCommand = new WinchUpCommand(winchSubsystem);

  XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  Joystick auxDriverController = new Joystick(OIConstants.AUXDRIVER_CONTROLLER_PORT);

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  /**
   * Prints "Gyro is calibrating..." and calibrates the gyro.
   */
  private void calibrate() {
    System.out.println("Gyro is calibrating...");
    driveSubsystem.getGyro().calibrate();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    calibrate();
    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    driveSubsystem.setDefaultCommand(new RunCommand(() -> {
      driveSubsystem.arcadeDrive(-driverController.getY(GenericHID.Hand.kLeft),
          driverController.getX(GenericHID.Hand.kRight));
    }, driveSubsystem));

  }

  // Use this method to define your button->command mappings.
  private void configureButtonBindings() {

    // shoot sequence
    new JoystickButton(driverController, XboxConstants.LEFT_STICK).whenPressed(() -> driveSubsystem.setMaxOutput(.5))
        .whenReleased(() -> driveSubsystem.setMaxOutput(1));
    new JoystickButton(driverController, XboxConstants.RIGHT_STICK).whenPressed(() -> driveSubsystem.setMaxOutput(.5))
        .whenReleased(() -> driveSubsystem.setMaxOutput(1));

    // new JoystickButton(driverController, XboxConstants.A_BUTTON);
    // new JoystickButton(driverController, XboxConstants.B_BUTTON);
    new JoystickButton(driverController, XboxConstants.X_BUTTON).whileHeld(intakeGateDownCommand);
    new JoystickButton(driverController, XboxConstants.Y_BUTTON).whileHeld(intakeGateUpCommand);
    new JoystickButton(driverController, XboxConstants.LB_BUTTON).whileHeld(elevatorDownCommand);
    new JoystickButton(driverController, XboxConstants.RB_BUTTON).whileHeld(elevatorUpCommand);

    // auxcommands\
    // new JoystickButton(auxDriverController, LogitecController.ONE_BUTTON);
    new JoystickButton(auxDriverController, LogitecController.FOUR_BUTTON).whileHeld(winchUpCommand);
    new JoystickButton(auxDriverController, LogitecController.TWO_BUTTON).whileHeld(autoAimCommand);
    new JoystickButton(auxDriverController, LogitecController.THREE_BUTTON)
        .whileHeld(new ShooterCommand(shooterSubsystem)
            .alongWith(new WaitCommand(1).andThen(new IndexerCommand(indexSubsystem))));
    new JoystickButton(auxDriverController, LogitecController.LB_BUTTON).whileHeld(intakeOutCommand);
    new JoystickButton(auxDriverController, LogitecController.RB_BUTTON).whileHeld(intakeInCommand);
    new JoystickButton(auxDriverController, LogitecController.RB_BUTTON)
        .whenPressed(() -> driveSubsystem.setMaxOutput(.4)).whenReleased(() -> driveSubsystem.setMaxOutput(1));

  }
  
  

  public Command getAutonomousCommand() {


    // return new FiveBallAuto(trajectoryPath, intakeGateSubsystem, intakeSubsystem, shooterSubsystem, indexSubsystem);
    // return new RedParallelAllyTrenchAuto(trajectoryPath, shooterSubsystem, indexSubsystem);
    return new testTrajectory(trajectoryPath);
  }

}
