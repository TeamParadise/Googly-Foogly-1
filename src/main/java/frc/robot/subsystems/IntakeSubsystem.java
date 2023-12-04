// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax intakeMotor;
  RelativeEncoder encoder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(MotorConstants.kIntakeMotor, MotorType.kBrushless);
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(15);
    encoder = intakeMotor.getEncoder();
  }

  public void Intake() {
    System.out.println ("Spinning in!");
    if (encoder.getVelocity() < 38){
      intakeMotor.set(0.25);
    } else intakeMotor.set(IntakeConstants.kSpinIn);

  }

  public void Outake() {
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.set(IntakeConstants.kSpinOut);
  }

  public void StopIntake() {
    System.out.println ("Stopping Intake");
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.set(0.1);
  }

  public void kEject() {
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.set(IntakeConstants.kEject);
  }

  public void setCoastMode(){
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode(){
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", intakeMotor.getAppliedOutput()); 
    SmartDashboard.putNumber("Intake Temp", ((intakeMotor.getMotorTemperature()*9)/5) + 32);
  }
}
