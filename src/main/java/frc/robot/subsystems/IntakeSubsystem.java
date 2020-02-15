/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  private final WPI_VictorSPX intakeMaster = new WPI_VictorSPX(IntakeConstants.INTAKE_MASTER);

  public IntakeSubsystem() {
    //if it is pushing it out with a positive value change this 
    intakeMaster.setInverted(false);
  }

  public void intakeVoltage(double voltage){
    intakeMaster.setVoltage(voltage);
  }

}
