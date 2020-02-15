/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.PIDConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class DriveSubsystem extends SubsystemBase implements Loggable {

                                                  //** HARDWARE **\\

  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(DriveConstants.LEFT_DRIVE_MASTER_CAN_ID);
  private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(DriveConstants.LEFT_DRIVE_SLAVE_CAN_ID);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(DriveConstants.RIGHT_DRIVE_MASTER_CAN_ID);
  private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(DriveConstants.RIGHT_DRIVE_SLAVE_CAN_ID);   
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final PowerDistributionPanel Pdp = new PowerDistributionPanel(MiscConstants.PDP_CAN_ID);

  
                                                   //**TRAJECTORY**\\
  DifferentialDriveOdometry diffDriveOdo;
  DifferentialDriveKinematics diffDriveKine = new DifferentialDriveKinematics(Units.inchesToMeters(25));
  SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(PIDConstants.KS_FEEDFOWARD, PIDConstants.KV_FEEDFOWARD, PIDConstants.KA_FEEDFOWARD);
  PIDController leftPIDController = new PIDController(9.31, 0, 4.51);
  PIDController rightPIDController = new PIDController(9.31, 0, 4.51);
  Pose2d pose;
 
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    leftMaster.configClosedloopRamp(1);
    rightMaster.configClosedloopRamp(1);
//check the ramp doesnt seem to work
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    leftMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    resetEncoders();

   diffDriveOdo = new DifferentialDriveOdometry(getHeading());

  }

 

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {

    //set to true so it is less sensitive at lower speeds
    differentialDrive.arcadeDrive(fwd, rot); //(fwd, rot, true);

  }

  public Rotation2d getHeading() {

    //negative because of the unit circle
    return Rotation2d.fromDegrees(-gyro.getAngle());

    }

  @Log
  public double getGyroAngle(){
    return gyro.getAngle();
  }

  /**
   * Returns the current of the specified channel in Amps
   * @param channel The channel of the pdp
   * @return the current of the channel given
   */
  double getPDPCurent(int channel){
    return  Pdp.getCurrent(channel);
  }


  @Log
  public double getLeftPosition(){

    return
      //1 or 10?
      //get rid of magic numbers
      leftMaster.getSelectedSensorPosition() * DriveConstants.INVERT_ENCODER * DriveConstants.TICKS_TO_REVOLUTIOIN_MAG_ENCODER  * 2
        * Math.PI * Units.inchesToMeters(DriveConstants.WHEEL_RADIUS);

  }

  @Log
  public double getRightPosition() {

    return
    // 1 or 10?
    // get rid of magic numbers
    //removedd the gear ratio because the position is is reading from the axel
    rightMaster.getSelectedSensorPosition(0) * DriveConstants.TICKS_TO_REVOLUTIOIN_MAG_ENCODER * 2
        * Math.PI * Units.inchesToMeters(DriveConstants.WHEEL_RADIUS);

  }

  public ADXRS450_Gyro getGyro(){
    return gyro;
  }

  
  public SimpleMotorFeedforward getFeedfoward() {

    return driveFeedforward;

  }



    /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {

   leftMaster.setSelectedSensorPosition(0, 0, 10);
   rightMaster.setSelectedSensorPosition(0, 0, 10);

  }

  public void resetGyro(){

    gyro.reset();
    
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  // public double getAverageEncoderDistance() {
  //   return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  // }
  public Pose2d getPose() {

    return pose;

  }

  public void setOutput(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts); 

    differentialDrive.feed();
  }

  public void setVoltageCompensation(boolean isEnabled, double volts) {
    leftMaster.configVoltageCompSaturation(volts);
    leftMaster.enableVoltageCompensation(isEnabled);

    rightMaster.configVoltageCompSaturation(volts);
    rightMaster.enableVoltageCompensation(isEnabled);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(0)  * DriveConstants.INVERT_ENCODER* DriveConstants.TICKS_TO_REVOLUTION_SECONDS_MAG_ENCODER  * 2 * Math.PI *
    Units.inchesToMeters(DriveConstants.WHEEL_RADIUS), rightMaster.getSelectedSensorVelocity(0)  * DriveConstants.TICKS_TO_REVOLUTION_SECONDS_MAG_ENCODER  * 2 * Math.PI *
    Units.inchesToMeters(DriveConstants.WHEEL_RADIUS));
  }

public DifferentialDriveKinematics getKinematics() {

    return diffDriveKine;
}

public PIDController getLeftPIDController(){

  return leftPIDController;

}

public PIDController getRightPIDController(){

  return rightPIDController;

}
/**
 * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
  *
  * @param maxOutput the maximum output to which the drive will be constrained
  */
public void setMaxOutput(double maxOutput) {

    differentialDrive.setMaxOutput(maxOutput);
    
}

  @Override
 public void periodic(){

 // gyroangle = Rotation2d.fromDegrees(-gyro.) 
 pose = diffDriveOdo.update(getHeading(), getLeftPosition(), getRightPosition());
 
 diffDriveOdo.update(getHeading(), getLeftPosition(), getRightPosition());
 
 

  SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
  SmartDashboard.getNumber("Gyro Setpoint", 0);
  SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  SmartDashboard.putNumber("Pdp current for channel " + MiscConstants.PDP_CHANNEL_0 , getPDPCurent(MiscConstants.PDP_CHANNEL_1));
  SmartDashboard.putNumber("Pdp current for channel " + MiscConstants.PDP_CHANNEL_1 , getPDPCurent(MiscConstants.PDP_CHANNEL_2));
  SmartDashboard.putNumber("Pdp current for channel " + MiscConstants.PDP_CHANNEL_2 , getPDPCurent(MiscConstants.PDP_CHANNEL_1));
  SmartDashboard.putNumber("Pdp current for channel " + MiscConstants.PDP_CHANNEL_3 , getPDPCurent(MiscConstants.PDP_CHANNEL_1));
  SmartDashboard.putNumber("Pdp current for channel " + MiscConstants.PDP_CHANNEL_12 , getPDPCurent(MiscConstants.PDP_CHANNEL_1));
  SmartDashboard.putNumber("Pdp current for channel " + MiscConstants.PDP_CHANNEL_13, getPDPCurent(MiscConstants.PDP_CHANNEL_1));
  SmartDashboard.putNumber("Pdp current for channel " + MiscConstants.PDP_CHANNEL_14 , getPDPCurent(MiscConstants.PDP_CHANNEL_1));
  
}
 
}
