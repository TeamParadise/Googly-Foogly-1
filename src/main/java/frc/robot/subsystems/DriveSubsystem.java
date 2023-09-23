// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.MotorConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;



public class DriveSubsystem extends SubsystemBase {
  
  //Initialize motors
  WPI_TalonSRX leftMotorMain = new WPI_TalonSRX(MotorConstants.kLeftMotorMain);
  WPI_TalonSRX leftMotorFollow = new WPI_TalonSRX(MotorConstants.kLeftMotorFollow);
  WPI_TalonSRX rightMotorMain = new WPI_TalonSRX(MotorConstants.kRightMotorMain);
  WPI_TalonSRX rightMotorFollow = new WPI_TalonSRX(MotorConstants.kRightMotorFollow);
    
  //Initialize Gyro 
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
 
  ShuffleboardTab pidtab = Shuffleboard.getTab("PID Tab");

  GenericEntry setpointWidget, errorWidget, positionWidget, leftDriveWidget, rightDriveWidget;
 ;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configMotors();
    resetGyro();
    configSmartDashboard();
  }

  public void configSmartDashboard() {
    setpointWidget = pidtab.add("Setpoint", 0.0).getEntry();
    errorWidget = pidtab.add("Error", 0.0).getEntry();
    positionWidget = pidtab.add("Position", 0.0).getEntry();
    leftDriveWidget = pidtab.add("Left Output", 0.0).getEntry();
    rightDriveWidget = pidtab.add("Right Output", 0.0).getEntry();
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

    leftMotorMain.configVoltageCompSaturation(12);
    leftMotorFollow.configVoltageCompSaturation(12);
    rightMotorFollow.configVoltageCompSaturation(12);
    rightMotorMain.configVoltageCompSaturation(12);

    

    System.out.println("Motors Configured!"); 
  }

  public void DriveArcade(double moveSpeed, double rotateSpeed) {
    double leftOutput = moveSpeed*(0.8) + rotateSpeed*(0.7);
    double rightOutput = moveSpeed*(0.8) - rotateSpeed*(0.7);
    leftMotorMain.set(ControlMode.PercentOutput, leftOutput);
    rightMotorMain.set(ControlMode.PercentOutput, rightOutput*.9);
  }

  public void DriveTank(double left, double right) {
    leftMotorMain.set(ControlMode.PercentOutput, left);
    rightMotorMain.set(ControlMode.PercentOutput, right*.9);
  }

  public void DriveTankPID(double left, double right, PIDController pid, double position) {
    leftMotorMain.set(ControlMode.PercentOutput, left);
    rightMotorMain.set(ControlMode.PercentOutput, right*.9);
 
    setpointWidget.setDouble(pid.getSetpoint());
    errorWidget.setDouble(pid.getPositionError());
    positionWidget.setDouble(position);
    leftDriveWidget.setDouble(left);
    rightDriveWidget.setDouble(right);
  }

  public void stopDrive() {
    leftMotorMain.set(ControlMode.PercentOutput, 0);
    rightMotorMain.set(ControlMode.PercentOutput, 0);
  }

  // public CommandBase setCoastMode() {
  //   return runOnce(() -> {
  //     leftMotorMain.setNeutralMode(NeutralMode.Coast);
  //     rightMotorMain.setNeutralMode(NeutralMode.Coast);
  //   });
  // }

  public CommandBase setBrakeMode() {
    return runOnce(() -> {
      leftMotorMain.setNeutralMode(NeutralMode.Brake);
      rightMotorMain.setNeutralMode(NeutralMode.Brake);
      leftMotorFollow.setNeutralMode(NeutralMode.Brake);
      rightMotorFollow.setNeutralMode(NeutralMode.Brake);
    });
  }

  //Move the following 3 to specific Gyro File
  public double getGyroAngle() {
    return ahrs.getAngle();
  }

  public double getGyroTilt() {
    return ahrs.getRoll();
  }

  public CommandBase resetGyro() {
    return runOnce(() -> { 
      ahrs.reset();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Maybe move these to specific "IO" File
    SmartDashboard.putNumber("Gyro Pitch", getGyroTilt());
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
    SmartDashboard.putNumber("Left Voltage", leftMotorMain.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Voltage", rightMotorMain.getMotorOutputVoltage());

    setBrakeMode();

    SmartDashboard.updateValues();
  }
}
