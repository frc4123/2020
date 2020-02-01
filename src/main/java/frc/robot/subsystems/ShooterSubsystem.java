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

  private final WPI_VictorSPX shooterMaster = new WPI_VictorSPX(ShooterConstants.SHOOTER_MASTER);
  private final WPI_VictorSPX shooterSlave = new WPI_VictorSPX(ShooterConstants.SHOOTER_SLAVE);

  private final SpeedControllerGroup shooterMotors = new SpeedControllerGroup(shooterMaster, shooterSlave);

  public ShooterSubsystem() {

    shooterSlave.setInverted(true);

  }

  public void shooterSpeed(double speed) {

    shooterMotors.set(speed);

  }
  /*
   * public double returnShooterSpeed() { return speed; }
   */

}
