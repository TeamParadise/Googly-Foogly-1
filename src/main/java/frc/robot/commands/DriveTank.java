// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveTank extends CommandBase {
  
  private double leftSpeed;
  private double rightSpeed;

  /** Creates a new DriveTank. */
  public DriveTank(double left, double right) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
    leftSpeed = left;
    rightSpeed = right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double left = RobotContainer.m_driverController.getLeftX()*-1;
    // double right = RobotContainer.m_driverController.getRightX();
    RobotContainer.m_driveSubsystem.DriveTank(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
