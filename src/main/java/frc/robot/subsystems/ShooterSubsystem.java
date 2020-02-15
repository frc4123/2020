/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */

  private final WPI_VictorSPX shooterTop = new WPI_VictorSPX(ShooterConstants.SHOOTER_MASTER);
  private final WPI_VictorSPX shooterBottom = new WPI_VictorSPX(ShooterConstants.SHOOTER_SLAVE);

  private final SpeedControllerGroup shooterMotors = new SpeedControllerGroup(shooterTop, shooterBottom);

  public ShooterSubsystem() {

    shooterBottom.setInverted(true);

  }

  public void shooterVoltage(double topVoltage, double bottomVoltage) {

    shooterTop.setVoltage(topVoltage);
    shooterBottom.setVoltage(bottomVoltage);

  }

  public void shooterVoltage(double voltage) {

    shooterMotors.setVoltage(voltage);

  }

  public WPI_VictorSPX getShooterMaster(){
    return shooterTop;
  }

  public WPI_VictorSPX getShooterSlave(){
    return shooterBottom;
  }

  /*
   * public double returnShooterSpeed() { return speed; }
   */

}
