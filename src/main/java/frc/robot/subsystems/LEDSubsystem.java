// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDSubsystem extends SubsystemBase {
  private CANdle candle = new CANdle(40);
  private final int LED_COUNT = 8; // Just the onboard LEDs
  private Animation currentAnimation = null;

  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.brightnessScalar = 1.0;
    config.vBatOutputMode = VBatOutputMode.Off;
    candle.configAllSettings(config);

    currentAnimation = new RainbowAnimation(1.0, 1.0, LED_COUNT);
  }

  @Override
  public void periodic() {
    candle.animate(currentAnimation);
  }
}
