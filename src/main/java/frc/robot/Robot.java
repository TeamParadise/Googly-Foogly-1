// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<Boolean> m_balance = new SendableChooser<>();
  private final SendableChooser<Boolean> m_taxi = new SendableChooser<>();
  private final SendableChooser<String> m_cargo = new SendableChooser<>();
  public static LEDSubsystem m_led = new LEDSubsystem();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
          
    
    RobotContainer.m_driveSubsystem.setBrakeMode();

    // CameraServer.startAutomaticCapture("USB Camera 0", 0);

    m_chooser.setDefaultOption("Mid", "kMid");
    m_chooser.addOption("High", "kHigh");
    m_chooser.addOption("None", "kNone");

    m_taxi.addOption("Yes", true);
    m_taxi.setDefaultOption("No", false);

    m_balance.addOption("Yes", true);
    m_balance.setDefaultOption("No", false);

    m_cargo.setDefaultOption("Cone", "kCone");
    m_cargo.addOption("Cube", "kCube");
  
    SmartDashboard.putData("Scoring Level", m_chooser);
    SmartDashboard.putData("Game Piece", m_cargo);
    SmartDashboard.putData("Taxi", m_taxi);
    SmartDashboard.putData("Balance", m_balance);
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // RobotContainer.m_EncoderPID.setCoastMode();
  }

  @Override
  public void disabledPeriodic() {
    // RobotContainer.m_EncoderPID.setCoastMode();
    RobotContainer.m_IntakeSubsystem.setBrakeMode();

    // System.out.println(// RobotContainer.m_EncoderPID.getEncoder());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_chooser.getSelected(), m_balance.getSelected(), m_taxi.getSelected());
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
     m_autonomousCommand.schedule();
    }
    // RobotContainer.m_EncoderPID.setBrakeMode();
    RobotContainer.m_driveSubsystem.setBrakeMode();
    RobotContainer.m_driveSubsystem.resetGyro();
  }
 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    RobotContainer.m_driveSubsystem.setBrakeMode();

    if (m_autonomousCommand != null) {
     m_autonomousCommand.cancel();
    }
    
    // RobotContainer.m_EncoderPID.setBrakeMode();
    RobotContainer.m_IntakeSubsystem.setBrakeMode();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.println("Motor Output: " + // RobotContainer.m_EncoderPID.getMeasurement());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}


