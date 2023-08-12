// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class MoveArmMid extends CommandBase {
  /** Creates a new MoveArmMid. */
  public MoveArmMid() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_EncoderPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_EncoderPID.setSetpoint(ArmConstants.kMidArm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = RobotContainer.m_EncoderPID.getMeasurement();
    RobotContainer.m_EncoderPID.useOutput(output, ArmConstants.kMidArm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_EncoderPID.moveArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
