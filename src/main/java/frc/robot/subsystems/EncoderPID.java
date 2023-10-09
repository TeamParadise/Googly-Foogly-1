// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Scanner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorConstants;

public class EncoderPID extends PIDSubsystem {

  private static final double kp = 0.02;
  private static final double ki = 0.00;
  private static final double kd = 0.006;

  public CANSparkMax sparkMax;
  public RelativeEncoder encoder;



  //Initialize Motor
  CANSparkMax armMotor;
  
  /** Creates a new ArmPID. */
  public EncoderPID() {
    super(
        // The PIDController used by the subsystem
        new PIDController(kp, ki, kd));
    armMotor = new CANSparkMax(MotorConstants.kArmMotor, MotorType.kBrushless);
    armMotor.setInverted(false);
    
    encoder = armMotor.getEncoder();
    this.getController().setTolerance(5.0);
    this.getController().setIntegratorRange(-5, 5);
    armMotor.setOpenLoopRampRate(0.35);
  }

  // public void setSetpoint(double setpoint) {
  //   this.setSetpoint(setpoint);
  // }

  public void resetPID() {
    this.resetPID();
  }

  public void moveArm(double speed) {
    armMotor.set(speed);
  }
  
  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    armMotor.set(output);
  }
  

  
  public CommandBase setCoastMode(){
    return runOnce(() -> {
      armMotor.setIdleMode(IdleMode.kCoast);
    });

  }

  public CommandBase setBrakeMode(){
  return runOnce(() -> {
      armMotor.setIdleMode(IdleMode.kBrake);
    });
  }
  

  public double getEncoder() {
    return armMotor.getEncoder().getPosition();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return this.getController().calculate(armMotor.getEncoder().getPosition());
  }

  //PID will stop when this returns true
  public boolean atSetpoint() {
    return this.getController().atSetpoint();
  }

}
