/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.AutoAngleCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final AutoAngleCommand autoAimCommand = new AutoAngleCommand(driveSubsystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem); 
  private final ElevatorCommand elevatorCommand = new ElevatorCommand(elevatorSubsystem);
  private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem);


  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  // aux driver
  Joystick auxDriverController = new Joystick(OIConstants.AUXDRIVER_CONTROLLER_PORT);

  /**
   * Prints "Gyro is calibrating..." and calibrates the gyro.
   */
  private void calibrate(){
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

    // Set the default drive command to split-stick arcade drive
    
    driveSubsystem
        .setDefaultCommand(new RunCommand(() -> driveSubsystem.arcadeDrive(-driverController.getY(GenericHID.Hand.kLeft),
            driverController.getX(GenericHID.Hand.kRight)), driveSubsystem));

    shooterSubsystem.setDefaultCommand(new RunCommand(
        () -> shooterSubsystem.shooterSpeed(driverController.getRawAxis(XboxConstants.LEFT_TRIGGER_AXIS)),
        shooterSubsystem));

 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
   

    // 100 percent intake
    new JoystickButton(driverController, Button.kB.value).whileHeld(shooterCommand);
    new JoystickButton(driverController, XboxConstants.A_BUTTON).whileHeld(autoAimCommand);
    new JoystickButton(driverController, XboxConstants.B_BUTTON).whileHeld(intakeCommand);
    new JoystickButton(driverController, XboxConstants.Y_BUTTON).whileHeld(elevatorCommand);
    
  }

  Trajectory trajectory;

  public Trajectory getTrajectory() {
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/PleaseStraight.wpilib.json"));
    }

    catch (IOException e) {
      e.printStackTrace();
    }
    return trajectory;
  }

  public Command getAutonomousCommand()
  {
    TrajectoryConfig config = new TrajectoryConfig(.25, .25).setKinematics(driveSubsystem.getKinematics());
      
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      
      List.of(
        new Translation2d(1,1),
        new Translation2d(2,-1)
        ),

      new Pose2d(3, 0, new Rotation2d(0)),
      
      config);

      //TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/SimplePath.wpilib.json"));
       
      RamseteCommand command = new RamseteCommand(
       trajectory,
       driveSubsystem::getPose,
       new RamseteController(2.0, 0.7),
       driveSubsystem.getFeedfoward(),
       driveSubsystem.getKinematics(),
       driveSubsystem::getWheelSpeeds,
       driveSubsystem.getLeftPIDController(),
       driveSubsystem.getRightPIDController(),
       driveSubsystem::setOutput,
       driveSubsystem 
      );

     
   
   //ramsete does not auto send a 0,0 to the output
    return command.andThen(() -> driveSubsystem.setOutput(0,0));
     
  }
  // auto chooser for SD??
  //return chooser.getSelected(); 
  
}
