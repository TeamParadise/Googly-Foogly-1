// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  WPI_VictorSPX armMotor;
  public ArmSubsystem() {
    armMotor = new WPI_VictorSPX(Constants.MotorConstants.kArmMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveArm(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }
}
