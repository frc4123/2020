/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import io.github.oblarg.oblog.annotations.Log;

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperSubsystem.
   */
  private final WPI_VictorSPX hopperMaster = new WPI_VictorSPX(HopperConstants.HOPPER_MASTER_CAN_ID);

  public HopperSubsystem() {
    hopperMaster.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Log
  public void hopperVoltage(double voltage){
    hopperMaster.setVoltage(voltage);
  }
}
