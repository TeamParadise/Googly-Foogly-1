// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax intakeMotor;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(MotorConstants.kIntakeMotor, MotorType.kBrushless);

    intakeMotor.setInverted(false);
  }

  public void Intake() {
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.set(0.5);
  }

  public void setCoastMode(){
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode(){
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void Outake() {
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.set(-0.6);
  }

  public void StopIntake() {
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.set(0.13);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
