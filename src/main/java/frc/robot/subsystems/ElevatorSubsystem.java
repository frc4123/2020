/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MiscConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ElevatorSubsystem extends SubsystemBase implements Loggable {
 
 private final WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(DriveConstants.ELEVATOR_MOTOR_CAN_ID);

 private final DigitalInput limitSwitch1 = new DigitalInput(MiscConstants.ELEVATOR_HIGH_SWITCH);
 private final DigitalInput limitSwitch2 = new DigitalInput(MiscConstants.ELEVATOR_LOW_SWITCH);

  public ElevatorSubsystem() {
    elevatorMotor.configOpenloopRamp(1);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Log
  public boolean isLimitTopSwitchHit(){
    return limitSwitch1.get();
  }

  @Log
  public boolean isLimitBottomSwitchHit(){
    return limitSwitch2.get();
  }

  @Log
  public void setVoltage(double voltage){
    elevatorMotor.setVoltage(voltage);
  }


}
