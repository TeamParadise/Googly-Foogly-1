// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveLevel extends CommandBase {
  /** Creates a new DriveLevel. */

  public PIDController tiltController;

  public DriveLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
    tiltController = new PIDController(0.005, 0, 0);
    tiltController.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tiltController.reset();
    System.out.println("Running Auto Balance!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_command = tiltController.calculate(RobotContainer.m_driveSubsystem.getGyroTilt());
    double right_command = tiltController.calculate(RobotContainer.m_driveSubsystem.getGyroTilt());
    RobotContainer.m_driveSubsystem.DriveTank(left_command*-1, right_command*-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping Auto Balance!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
