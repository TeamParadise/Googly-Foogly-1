// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static class MotorConstants {
    public static final int kLeftMotorMain = 1;
    public static final int kLeftMotorFollow = 2;
    public static final int kRightMotorMain = 3;
    public static final int kRightMotorFollow = 4;

    //Arm
    public static final int kArmMotor = 21;

    //Intake
    public static final int kIntakeMotor = 32;


  }

  public static class ArmConstants {
    public static final double kSVolts = 0;
    public static final double kVVoltSecondsPerRotation = 0;

    public static final double kBottom = 5;
    public static final double kLowArm = 25;
    public static final double kMidArm = 80;
    public static final double kHighArm = 100 ;
  }

  public static class IntakeConstants {

    public static final double kSpinIn = 0.5;
    public static final double kSpinOut = -0.6;
    public static final double kHold = 0.13;
    public static final double kEject = -1;

  }

}
