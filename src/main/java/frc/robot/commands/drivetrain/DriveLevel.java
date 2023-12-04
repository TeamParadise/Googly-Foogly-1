// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveLevel extends CommandBase {
  /** Creates a new DriveLevel. */

  public PIDController tiltController;

  public DriveLevel() {
    // Use addRequirements() here to declare subsystem dependencies.
    tiltController = new PIDController(0.01, 0, 0.0015);
    tiltController.setSetpoint(45);
    tiltController.setTolerance(4 );
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
    double left_command = MathUtil.clamp(tiltController.calculate(RobotContainer.m_driveSubsystem.getGyroTilt()), -0.4, 0.4);
    RobotContainer.m_driveSubsystem.DriveTankPID(left_command, left_command, tiltController, RobotContainer.m_driveSubsystem.getGyroTilt());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping Auto Balance!");
    RobotContainer.m_driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
