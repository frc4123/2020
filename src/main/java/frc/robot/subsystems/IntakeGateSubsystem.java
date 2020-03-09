/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeGateSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeGateSubsystem.
   */
  private final WPI_TalonSRX intakeGate = new WPI_TalonSRX(IntakeConstants.INTAKE_GATE_CAN_ID);
  public IntakeGateSubsystem() {
    intakeGate.setNeutralMode(NeutralMode.Brake);
  }

  public void setIntakeGateVoltage(double voltage) {
    intakeGate.setVoltage(voltage);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
