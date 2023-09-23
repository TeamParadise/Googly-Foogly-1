// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveArmHigh;
import frc.robot.commands.Outake;
import frc.robot.commands.StopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHighTaxi extends SequentialCommandGroup {
  /** Creates a new ShootHighTaxi. */
  public ShootHighTaxi() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new MoveArmHigh(), new StopIntake()).withTimeout(4),
      new Outake().withTimeout(1),
      new StopIntake()
      // new DriveTank(-0.3, -0.3).withTimeout(1),
      // new MoveArmLow()
      // new ParallelCommandGroup(new DriveTank(-0.3, 0.3), new MoveArmLow()).withTimeout(3)
    );
  }
}
