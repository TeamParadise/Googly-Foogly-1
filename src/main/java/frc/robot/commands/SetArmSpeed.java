// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetArmSpeed extends CommandBase {
  /** Creates a new SetArmSpeed. */
  public SetArmSpeed() {
    
    addRequirements(RobotContainer.m_EncoderPID);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_driverController.getRightTriggerAxis() > 0.1) {
      RobotContainer.m_EncoderPID.setCoastMode();
      RobotContainer.m_EncoderPID.moveArm(0.7 * RobotContainer.m_driverController.getRightTriggerAxis());
    } else if (RobotContainer.m_driverController.getLeftTriggerAxis() > 0.1) {
      RobotContainer.m_EncoderPID.setCoastMode();
      RobotContainer.m_EncoderPID.moveArm(-0.7 * RobotContainer.m_driverController.getLeftTriggerAxis());
    } else {
      RobotContainer.m_EncoderPID.setBrakeMode();
      RobotContainer.m_EncoderPID.moveArm(0);
    }
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
