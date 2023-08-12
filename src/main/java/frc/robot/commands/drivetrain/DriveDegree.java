// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveDegree extends CommandBase {
  /** Creates a new DriveDegree. */

  PIDController degreeController;

  public DriveDegree(double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
    degreeController = new PIDController(0.0025, 0, 0);
    degreeController.setSetpoint(setpoint);
    degreeController.enableContinuousInput(-180, 180);
    degreeController.setTolerance(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    degreeController.reset();
    System.out.println("Starting Turn!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left_command = MathUtil.clamp(degreeController.calculate(RobotContainer.m_driveSubsystem.getGyroAngle()), -0.5, 0.5);
    double right_command =  MathUtil.clamp(degreeController.calculate(RobotContainer.m_driveSubsystem.getGyroAngle()), -0.5, 0.5);
    RobotContainer.m_driveSubsystem.DriveTankPID(left_command*-1, right_command, degreeController, RobotContainer.m_driveSubsystem.getGyroAngle());
      //  RobotContainer.m_driveSubsystem.DriveTank(left_command*-1, right_command);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.stopDrive();
    System.out.println("Stopping Turn!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
