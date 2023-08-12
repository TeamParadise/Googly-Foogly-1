// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmHigh;
import frc.robot.commands.arm.MoveArmMid;
import frc.robot.commands.drivetrain.DriveTank;
import frc.robot.commands.intake.Outake;
import frc.robot.commands.intake.StopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreMid extends SequentialCommandGroup {
  /** Creates a new ShootMid. */
  public ScoreMid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new MoveArmHigh(), new StopIntake()).withTimeout(1.85),
      new DriveTank(0.12, 0.12).withTimeout(1),
      RobotContainer.m_EncoderPID.setCoastMode(),
      new WaitCommand(1),
      new Outake().withTimeout(0.2),
      RobotContainer.m_EncoderPID.setBrakeMode(),
      new StopIntake()
    );
  }
}
