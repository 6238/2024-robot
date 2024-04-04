// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Map;
import java.util.Optional;

import static java.util.Map.entry;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

public class LEDSubsystem extends SubsystemBase {
  private CANdle candle = new CANdle(40);
  private final int LED_COUNT = 8; // Just the onboard LEDs
  private LEDMode currentMode = null;
  private LEDMode lastSentMode = null;

  public LEDSubsystem() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.brightnessScalar = 1.0;
    config.vBatOutputMode = VBatOutputMode.Off;
    candle.configAllSettings(config);

    currentMode = LEDMode.RAINBOW;
  }

  @Override
  public void periodic() {
    if (currentMode != lastSentMode) {
      candle.animate(MODES.get(currentMode));
      lastSentMode = currentMode;
    }
    

    // If disabled, we want to do the alliance pulse or default to rainbow
    if (DriverStation.isDisabled()) {
      this.setAnimation(this.getAllianceAnimation(DriverStation.getAlliance()));
    }
  }

  public void setAnimation(LEDMode mode) {
    currentMode = mode;
  }

  private LEDMode getAllianceAnimation(Optional<Alliance> ally) {
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return LEDMode.RED_SLOW_PULSE;
      } else {
        return LEDMode.BLUE_SLOW_PULSE;
      }
    } else {
      return LEDMode.RAINBOW;
    }
  }

  public Command setAnimationToAllianceColorCommand(Optional<Alliance> ally) {
    return runOnce(() -> {
      LEDMode anim = getAllianceAnimation(ally);
      currentMode = anim;
    }).ignoringDisable(true);
  }

  public Command indicateNeedNoteCommand() {
    return runOnce(() -> {
      currentMode = LEDMode.ORANGE_QUICK_PULSE;
    });
  }

  public Command indicateIntookCommand() {
    return runOnce(() -> {
      currentMode = LEDMode.PURPLE_CHASE;
    });
  }

  public enum LEDMode {
    RED_SLOW_PULSE,
    BLUE_SLOW_PULSE,
    RAINBOW,
    ORANGE_QUICK_PULSE, 
    PURPLE_CHASE
  }

  private final Map<LEDMode, Animation> MODES = Map.ofEntries(
    entry(LEDMode.RED_SLOW_PULSE, new SingleFadeAnimation(255, 0, 0, 0, 0.5, LED_COUNT, 0)),
    entry(LEDMode.BLUE_SLOW_PULSE, new SingleFadeAnimation(0, 0, 255, 0, 0.5, LED_COUNT, 0)),
    entry(LEDMode.RAINBOW, new RainbowAnimation(1.0, 1.0, LED_COUNT)),
    entry(LEDMode.ORANGE_QUICK_PULSE, new SingleFadeAnimation(255, 165, 0, 0, 1, LED_COUNT, 0)),
    entry(LEDMode.PURPLE_CHASE, new ColorFlowAnimation(255, 0, 255, 0, 0.75, LED_COUNT, Direction.Forward))
  );
}
