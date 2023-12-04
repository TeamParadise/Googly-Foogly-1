// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.RobotContainer;
import frc.robot.Constants.MotorConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;



public class DriveSubsystem extends SubsystemBase {
  
  //Initialize motors
  // CANSparkMax leftMotorMain = new CANSparkMax(MotorConstants.kLeftMotorMain, MotorType.kBrushless);
  // CANSparkMax leftMotorFollow = new CANSparkMax(MotorConstants.kLeftMotorFollow, MotorType.kBrushless);
  // CANSparkMax rightMotorMain = new CANSparkMax(MotorConstants.kRightMotorMain, MotorType.kBrushless);
  // CANSparkMax rightMotorFollow = new CANSparkMax(MotorConstants.kRightMotorFollow, MotorType.kBrushless);

  TalonSRX leftMotorMain = new TalonSRX(MotorConstants.kLeftMotorMain);
  TalonSRX leftMotorFollow = new TalonSRX(MotorConstants.kLeftMotorFollow);
  TalonSRX rightMotorMain = new TalonSRX(MotorConstants.kRightMotorMain);
  TalonSRX rightMotorFollow = new TalonSRX(MotorConstants.kRightMotorFollow);

    
  //Initialize Gyro 
  AHRS ahrs = new AHRS(SPI.Port.kMXP);
 
  ShuffleboardTab pidtab = Shuffleboard.getTab("PID Tab");

  GenericEntry setpointWidget, errorWidget, positionWidget, leftDriveWidget, rightDriveWidget;

  double speedMultiplier = 1;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    configMotors();
    resetGyro();
    configSmartDashboard();
    setBrakeMode();
  }

  public void configSmartDashboard() {
    setpointWidget = pidtab.add("Setpoint", 0.0).getEntry();
    errorWidget = pidtab.add("Error", 0.0).getEntry();
    positionWidget = pidtab.add("Position", 0.0).getEntry();
    leftDriveWidget = pidtab.add("Left Output", 0.0).getEntry();
    rightDriveWidget = pidtab.add("Right Output", 0.0).getEntry();
  } 

  private void configMotors() {
    // leftMotorMain.setSmartCurrentLimit(65);
    // leftMotorFollow.setSmartCurrentLimit(65);
    // rightMotorMain.setSmartCurrentLimit(65);
    // rightMotorFollow.setSmartCurrentLimit(65);
    // leftMotorFollow.follow(leftMotorMain, false);
    // rightMotorFollow.follow(rightMotorMain, false);
    leftMotorFollow.follow(leftMotorMain);
    rightMotorFollow.follow(rightMotorMain);

    leftMotorMain.setInverted(true);
    leftMotorFollow.setInverted(true);
    rightMotorMain.setInverted(false);
    rightMotorFollow.setInverted(false);
     
    // leftMotorMain.setClosedLoopRampRate(0.1);
    // leftMotorFollow.setClosedLoopRampRate(0.1);
    // rightMotorMain.setClosedLoopRampRate(0.1);
    // rightMotorFollow.setClosedLoopRampRate(0.1)  ;

    // NEED A SPARK MAX EQUIVALENT
    // leftMotorMain.configVoltageCompSaturation(12);
    // leftMotorFollow.configVoltageCompSaturation(12);
    // rightMotorFollow.configVoltageCompSaturation(12);
    //rightMotorMain.configVoltageCompSaturation(12);

    

    System.out.println("Motors Configured!"); 
  }

  public void DriveArcade(double moveSpeed, double rotateSpeed) {
    double leftOutput = (moveSpeed*(speedMultiplier) + rotateSpeed) * Math.abs(speedMultiplier);
    double rightOutput = (moveSpeed*(speedMultiplier) - rotateSpeed) *  Math.abs(speedMultiplier);
    leftMotorMain.set(ControlMode.PercentOutput, (MathUtil.applyDeadband(leftOutput, 0.06)));
    rightMotorMain.set(ControlMode.PercentOutput, (MathUtil.applyDeadband(rightOutput, 0.06)));
  }

  public void DriveTank(double left, double right) {
    leftMotorMain.set(ControlMode.PercentOutput, left);
    rightMotorMain.set(ControlMode.PercentOutput, right);
  }

  public void DriveTankPID(double left, double right, PIDController pid, double position) {
    leftMotorMain.set(ControlMode.PercentOutput, left);
    rightMotorMain.set(ControlMode.PercentOutput, right);

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

  // public CommandBase setBrakeMode() {
  //   return runOnce(() -> {
  //     leftMotorMain.setIdleMode(IdleMode.kBrake);
  //     rightMotorMain.setIdleMode(IdleMode.kBrake);
  //     leftMotorFollow.setIdleMode(IdleMode.kBrake);
  //     rightMotorFollow.setIdleMode(IdleMode.kBrake);
  //   });
  // }

  public CommandBase halfSpeed(){
    return runOnce(() -> {
      if (Math.abs(speedMultiplier) < 1) speedMultiplier = Math.abs(speedMultiplier)/speedMultiplier;
      else speedMultiplier *= 0.65;
    });
  }

  public void setBrakeMode() {
    leftMotorMain.setNeutralMode(NeutralMode.Brake);
    leftMotorFollow.setNeutralMode(NeutralMode.Brake);
    rightMotorMain.setNeutralMode(NeutralMode.Brake);
    rightMotorFollow.setNeutralMode(NeutralMode.Brake);
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
    SmartDashboard.putNumber("Left Voltage", leftMotorMain.getBusVoltage());
    SmartDashboard.putNumber("Right Voltage", rightMotorMain.getBusVoltage());

    setBrakeMode();

    SmartDashboard.updateValues();
  }
}
