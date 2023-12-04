// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Robot;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.SetArmSpeed;
import frc.robot.commands.auto.Balance;
import frc.robot.commands.auto.Taxi;
import frc.robot.commands.drivetrain.DriveArcade;
import frc.robot.commands.intake.Intake;
import frc.robot.commands.intake.Outake;
import frc.robot.commands.intake.StopIntake;
// import frc.robot.commands.led.SetColor;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LEDSubsystem;
import java.nio.channels.SelectionKey;

import javax.print.attribute.standard.RequestingUserName;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final static ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  // public final static LEDSubsystem m_ledsubsystem = new LEDSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final static CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public final static CommandXboxController m_coDriverController =
      new CommandXboxController(OperatorConstants.kCoDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driveSubsystem.setDefaultCommand(new DriveArcade());
    m_armSubsystem.setDefaultCommand(new SetArmSpeed());
    // m_ledsubsystem.setDefaultCommand(new SetColor(-0.99));

  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // new JoystickButton(m_driverController, 4).onTrue(new MoveArmHigh());
    // new JoystickButton(m_driverController, 2).onTrue(new MoveArmMid());
    // new JoystickButton(m_driverController, 3).onTrue(new MoveArmLow());
    // new JoystickButton(m_driverController, 1).onTrue(new RestArm());
    // new JoystickButton(m_coDriverController, 1).onTrue(new Intake());
    // new JoystickButton(m_coDriverController, 3).onTrue(new Outake());
    // new JoystickButton(m_coDriverController, 2).onTrue(new StopIntake());

    m_coDriverController.y().onTrue(m_driveSubsystem.halfSpeed());

    m_coDriverController.a().onTrue(new Intake());
    m_coDriverController.x().onTrue(new Outake());
    m_coDriverController.b().onTrue(new StopIntake());
    // m_coDriverController.rightTrigger().onTrue(new SetColor(0.69));
    // m_coDriverController.leftTrigger().onTrue(new SetColor(0.91));
    // new Trigger(m_driverController.a(null)).onTrue(new Intake());

    // new Trigger (m_coDriverController ).onTrue(new StopIntake());


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String auto, boolean balance, boolean taxi) {
    // An example command will be run in autonomous
    
    // if (auto.equals("kHigh")) {
    //   if (taxi) return new ScoreHigh().withTimeout(7).andThen(new Taxi());
    //   else if (balance) return new ScoreHigh().withTimeout(6).andThen(new Balance());
    //   return new ScoreHigh();
    // }
    // else if (auto.equals("kMid")) {
    //   if (taxi) return new ScoreMid().withTimeout(7).andThen(new Taxi());
    //   else if (balance) return new ScoreMid().withTimeout(5).andThen(new Balance());
    //   return new ScoreMid();
    // }
    // else if (auto.equals("kNone")) {
    //   if (taxi) return new Taxi();
    //   if (balance) return new Balance();
    // }

    return null;
  } 
}
