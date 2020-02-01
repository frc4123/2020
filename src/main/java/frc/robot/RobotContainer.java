/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
// import java.io.IOException;
import java.nio.file.Paths;
// import java.util.Arrays;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
//import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.AutoAimCommand;
//import frc.robot.commands.ComplexAutoCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.HatchSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public AutoAimCommand autoAimCommand;

  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  // aux driver
  Joystick auxDriverController = new Joystick(OIConstants.AUXDRIVER_CONTROLLER_PORT);

  // The autonomous routines

  // A simple auto routine that drives forward a specified distance, and then
  // stops.

  // private double globalTime = Timer.getFPGATimestamp();

  /*
   * private final Command simpleAuto = new StartEndCommand( // Start driving
   * forward at the start of the command () ->
   * robotDrive.arcadeDrive(AutoConstants.kAutoDriveSpeed, 0), // Stop driving at
   * the end of the command () -> robotDrive.arcadeDrive(0, 0), // Requires the
   * drive subsystem robotDrive) // End the command when the robot's driven
   * distance exceeds the desired value // .withInterrupt( // () -> globalTime -
   * robot.robotInit().autoStartTime >= 3); // //how do we get start time
   * 
   * 
   * // A complex auto routine that drives forward, drops a hatch, and then drives
   * // backward. private final Command m_complexAuto = new
   * ComplexAutoCommand(robotDrive, m_hatchSubsystem);
   */

  // A chooser for autonomous commands
  // SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // Set the default drive command to split-stick arcade drive

    robotDrive
        .setDefaultCommand(new RunCommand(() -> robotDrive.arcadeDrive(-driverController.getY(GenericHID.Hand.kLeft),
            driverController.getX(GenericHID.Hand.kRight)), robotDrive));

    shooterSubsystem.setDefaultCommand(new RunCommand(
        () -> shooterSubsystem.shooterSpeed(driverController.getRawAxis(XboxConstants.LEFT_TRIGGER_AXIS)),
        shooterSubsystem));

    // Add commands to the autonomous command chooser
    // m_chooser.addOption("Simple Auto", simpleAuto);
    // m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    // Shuffleboard.getTab("Autonomous").add(chooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Grab the hatch when the 'A' button is pressed.
    /*
     * new JoystickButton(driverController, Button.kA.value) .whenPressed(new
     * InstantCommand(m_hatchSubsystem::grabHatch, m_hatchSubsystem)); // Release
     * the hatch when the 'B' button is pressed. new
     * JoystickButton(driverController, Button.kB.value) .whenPressed(new
     * InstantCommand(m_hatchSubsystem::releaseHatch, m_hatchSubsystem)); // While
     * holding the shoulder button, drive at half speed new
     * JoystickButton(driverController, Button.kBumperRight.value) .whenPressed(()
     * -> robotDrive.setMaxOutput(0.5)) .whenReleased(() ->
     * robotDrive.setMaxOutput(1));
     */
    // sets the shooter motor to 80 percent speed
    /*
     * new JoystickButton(driverController, Button.kX.value).whenPressed(() ->
     * shooterSubsystem.shooterSpeed(0.8)) .whenReleased(() ->
     * shooterSubsystem.shooterSpeed(0.0)); // sets the shooter motor to 90 percent
     * speed new JoystickButton(driverController, Button.kY.value).whenPressed(() ->
     * shooterSubsystem.shooterSpeed(0.9)) .whenReleased(() ->
     * shooterSubsystem.shooterSpeed(0.0)); // 100 percent speed new
     * JoystickButton(driverController, Button.kB.value).whenPressed(() ->
     * shooterSubsystem.shooterSpeed(1.0)) .whenReleased(() ->
     * shooterSubsystem.shooterSpeed(0.0));
     * 
     */
    // half speed intake

    driverController.getRawAxis(XboxConstants.LEFT_TRIGGER_AXIS);

    new JoystickButton(auxDriverController, Button.kA.value).whenPressed(() -> intakeSubsystem.intakeSpeed(0.5))
        .whenReleased(() -> intakeSubsystem.intakeSpeed(0.0));

    // 100 percent intake
    new JoystickButton(auxDriverController, Button.kB.value).whenPressed(() -> intakeSubsystem.intakeSpeed(1.0))
        .whenReleased(() -> intakeSubsystem.intakeSpeed(0.0));

    new JoystickButton(driverController, XboxConstants.X_BUTTON).whenPressed(new AutoAimCommand(robotDrive));

    // new JoystickButton(driverController, Button.kX.value).whenPressed(() -> {
    // autoAimCommand = new AutoAimCommand(robotDrive);
    // autoAimCommand.schedule();
    // }).whenReleased(() -> {

    // if (autoAimCommand != null) {

    // autoAimCommand.cancel();

    // }
    // });

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  Trajectory trajectory;
  public Trajectory getTrajectory() {
    try{
    trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PleaseStraight.wpilib.json"));
    }

    catch (IOException e)
    {
      e.printStackTrace();
    }
    return trajectory;
  }
  public Command getAutonomousCommand()
  {
    TrajectoryConfig config = new TrajectoryConfig(.25, .25);

    config.setKinematics(robotDrive.getKinematics());

    
  
      
      //Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/SimplePath.wpilib.json"));
      
  
 
    // if it compiles with errors comment out the above line and uncomment the bellow lijne (after the variable declaration)
    
    // TrajectoryGenerator.generateTrajectory(
    // Arrays.asList(new Pose2d(), new Pose2d(2.0, 0, new Rotation2d())),
    // config);
    
 
      RamseteCommand command = new RamseteCommand(
       getTrajectory(),
       robotDrive::getPose,
       new RamseteController(2.0, 0.7),
       robotDrive.getFeedfoward(),
       robotDrive.getKinematics(),
       robotDrive::getWheelSpeeds,
       robotDrive.getLeftPIDController(),
       robotDrive.getRightPIDController(),
       robotDrive::setOutput,
       robotDrive 
      );

     
   
   
    return command;
     
  }
  // auto chooser for SD??
  //return chooser.getSelected(); 
  
}
