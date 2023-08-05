// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class DriveSubsystem extends SubsystemBase {
  
  //Initialize motors
  TalonSRX leftMotorMain = new TalonSRX(MotorConstants.kLeftMotorMain);
  TalonSRX leftMotorFollow = new TalonSRX(MotorConstants.kLeftMotorFollow);
  TalonSRX rightMotorMain = new TalonSRX(MotorConstants.kRightMotorMain);
  TalonSRX rightMotorFollow = new TalonSRX(MotorConstants.kRightMotorFollow);
  



  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configMotors();
  }

  private void configMotors() {
    leftMotorFollow.set(ControlMode.Follower, MotorConstants.kLeftMotorMain);
    rightMotorFollow.set(ControlMode.Follower, MotorConstants.kRightMotorMain);

    leftMotorMain.setInverted(true);
    leftMotorFollow.setInverted(true);
    rightMotorMain.setInverted(false);
    rightMotorFollow.setInverted(false);
     
    leftMotorMain.configClosedloopRamp(0.25);
    leftMotorFollow.configClosedloopRamp(0.25);
    rightMotorMain.configClosedloopRamp(0.25);
    rightMotorFollow.configClosedloopRamp(0.25);

    System.out.println("Motors Configured!"); 
  }


  public void DriveArcade(double moveSpeed, double rotateSpeed) {
    double leftOutput = moveSpeed*(0.8) + rotateSpeed*(0.7);
    double rightOutput = moveSpeed*(0.8) - rotateSpeed*(0.7);
    leftMotorMain.set(ControlMode.PercentOutput, leftOutput);
    rightMotorMain.set(ControlMode.PercentOutput, rightOutput);
  }

  public void DriveTank(double left, double right) {
    leftMotorMain.set(ControlMode.PercentOutput, left);
    rightMotorMain.set(ControlMode.PercentOutput, right);
  }

  public void stopDrive() {
    leftMotorMain.set(ControlMode.PercentOutput, 0);
    rightMotorMain.set(ControlMode.PercentOutput, 0);
  }

  public void setCoastMode() {
    leftMotorMain.setNeutralMode(NeutralMode.Coast);
    rightMotorMain.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrakeMode() {
    leftMotorMain.setNeutralMode(NeutralMode.Brake);
    rightMotorMain.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
