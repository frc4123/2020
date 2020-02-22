/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
// import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.controller.RamseteController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PS4ButtonConstants;
import frc.robot.Constants.XboxConstants;
import frc.robot.commands.AutoAngleCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorUpCommand;
// import frc.robot.commands.IndexToShooter;
import frc.robot.commands.IndexWheelCommand;
import frc.robot.commands.IntakeInCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.IntakeGateDownCommand;
import frc.robot.commands.IntakeGateUpCommand;
// import frc.robot.commands.ShooterCommand;
import frc.robot.commands.WinchDownCommand;
import frc.robot.commands.WinchUpCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WinchSubsystem;

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
  // private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final WinchSubsystem winchSubsystem = new WinchSubsystem();
  private final HopperSubsystem hopperSubsystem = new HopperSubsystem();

  private final AutoAngleCommand autoAimCommand = new AutoAngleCommand(driveSubsystem);
  private final ElevatorUpCommand elevatorUpCommand = new ElevatorUpCommand(elevatorSubsystem);
  private final ElevatorDownCommand elevatorDownCommand = new ElevatorDownCommand(elevatorSubsystem);
  // private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem);
  private final WinchDownCommand winchDownCommand = new WinchDownCommand(winchSubsystem);
  private final WinchUpCommand winchUpCommand = new WinchUpCommand(winchSubsystem);
  private final IntakeGateDownCommand intakeGateDownCommand = new IntakeGateDownCommand(intakeSubsystem);
  private final IntakeGateUpCommand intakeGateUpCommand = new IntakeGateUpCommand(intakeSubsystem);
  private final IntakeInCommand intakeInCommand = new IntakeInCommand(intakeSubsystem);
  private final IntakeOutCommand intakeOutCommand = new IntakeOutCommand(intakeSubsystem);
  private final IndexWheelCommand indexCommand = new IndexWheelCommand(hopperSubsystem);
  //private final IndexToShooter indexToShooterCommand = new IndexToShooter(hopperSubsystem,shooterSubsystem);

  public DriveSubsystem getDriveSubsystem(){
    return driveSubsystem;
  }

  // The driver's controller
  XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  // aux driver
  Joystick auxDriverController = new Joystick(OIConstants.AUXDRIVER_CONTROLLER_PORT);

  /**
   * Prints "Gyro is calibrating..." and calibrates the gyro.
   */
  //put in robot init so it calibrates on turn on... maybe we want that?
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

 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  
   

    
    new JoystickButton(driverController, XboxConstants.LB_BUTTON).whileHeld(intakeInCommand);
    new JoystickButton(driverController, XboxConstants.RB_BUTTON).whileHeld(intakeOutCommand);
    new JoystickButton(driverController, PS4ButtonConstants.X_BUTTON).whileHeld(autoAimCommand);
    new JoystickButton(driverController, PS4ButtonConstants.SQUARE_BUTTON).whileHeld(intakeGateDownCommand);
    new JoystickButton(driverController, PS4ButtonConstants.TRIANGLE_BUTTON).whileHeld(intakeGateUpCommand);
    //new JoystickButton(driverController, XboxConstants.B_BUTTON).whileHeld(shooterCommand);
    //give elevator to aux. make intake roller triggers. make 
    //driver
    //    triggers -> intake roller
    //    bumpers -> winch
    //    x is auto aim
    //    intake gate square and triangle
    //aux
    //    manual shoot ->  

    //also can be one button pressed until the limit swtich is hit
    new JoystickButton(auxDriverController, XboxConstants.X_BUTTON).whenPressed(indexCommand.withTimeout(.5));
    new JoystickButton(auxDriverController, XboxConstants.LB_BUTTON).whileHeld(elevatorUpCommand);
    new JoystickButton(auxDriverController, XboxConstants.RB_BUTTON).whileHeld(elevatorDownCommand);
    new JoystickButton(auxDriverController, XboxConstants.A_BUTTON).whileHeld(winchDownCommand);
    new JoystickButton(auxDriverController, XboxConstants.B_BUTTON).whileHeld(winchUpCommand);
    
  }

  Trajectory trajectory;

  

  public Command getAutonomousCommand() {
    
    var autoVoltageConstraint =
     new DifferentialDriveVoltageConstraint(
        DriveConstants.SIMPLE_MOTOR_FEED_FOWARD,
        DriveConstants.DRIVE_KINEMATICS,
        DriveConstants.MAX_VOLTAGE_AUTO);

    TrajectoryConfig config =

      new TrajectoryConfig(DriveConstants.MAX_METERS_PER_SECOND,
                         DriveConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.DRIVE_KINEMATICS)
        .addConstraint(autoVoltageConstraint);
      
    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)), 
      
    //   List.of(
    //     new Translation2d(1,1),
    //     new Translation2d(2,-1)
    //     ),

    //   new Pose2d(3, 0, new Rotation2d(0)),
      
    //   config);
    try{

    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/SimplePath.wpilib.json"));
       
      RamseteCommand command = new RamseteCommand(
       trajectory,
       driveSubsystem::getPose,
       new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
       driveSubsystem.getFeedfoward(),
       DriveConstants.DRIVE_KINEMATICS,
       driveSubsystem::getWheelSpeeds,
       driveSubsystem.getLeftPIDController(),
       driveSubsystem.getRightPIDController(),
       driveSubsystem::setOutput,
       driveSubsystem 
      );

     
   
   //ramsete does not auto send a 0,0 to the output
    return command.andThen(() -> driveSubsystem.setOutput(0,0));

    } catch(IOException ex){
      System.out.println("Unable to open trajectory" );

    }
    
    return null;

  }
  // auto chooser for SD??
  //return chooser.getSelected(); 
  
}
