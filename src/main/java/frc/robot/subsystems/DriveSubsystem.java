// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;



public class DriveSubsystem extends SubsystemBase {
  
  //Initialize motors
  WPI_TalonSRX leftMotorMain = new WPI_TalonSRX(MotorConstants.kLeftMotorMain);
  WPI_TalonSRX leftMotorFollow = new WPI_TalonSRX(MotorConstants.kLeftMotorFollow);
  WPI_TalonSRX rightMotorMain = new WPI_TalonSRX(MotorConstants.kRightMotorMain);
  WPI_TalonSRX rightMotorFollow = new WPI_TalonSRX(MotorConstants.kRightMotorFollow);
  
  //Initialize Gyro 
  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configMotors();
    calibrateGyro();
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

    SmartDashboard.putNumber("Left Speed", leftOutput);
    SmartDashboard.putNumber("Right Speed", rightOutput);
  }

  public void DriveTank(double left, double right) {
    leftMotorMain.set(ControlMode.PercentOutput, left);
    rightMotorMain.set(ControlMode.PercentOutput, right);
    SmartDashboard.putNumber("Left Speed", right);
    SmartDashboard.putNumber("Right Speed", left);
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

  public double getGyroAngle() {
    return ahrs.getAngle();
  }

  public double getGyroTilt() {
    return ahrs.getPitch();
  }

  public void calibrateGyro() {
    ahrs.calibrate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro Pitch", getGyroTilt());
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.updateValues();
  }
}
