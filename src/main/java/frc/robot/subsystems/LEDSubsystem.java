// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED ledStrip = new AddressableLED(3);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);



  /** Creates a new LedSubsystem. */
  public LEDSubsystem() {
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public void setColor(double pwmValue) {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
