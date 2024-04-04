// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import static java.util.Map.entry;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IDs;
import frc.robot.Constants.OuttakeGains;
import frc.robot.Constants.OuttakeGainsAmp;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.telemetry.Alert;
import frc.robot.telemetry.Alert.AlertType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.OuttakeGainsAmp;

public class AmpSubsystem extends SubsystemBase {
  public double setpoint = 80.0;
  private CANSparkBase angleMotor;
  private SparkPIDController angle_pidController;
  private RelativeEncoder angle_encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private Alert motorFailed = new Alert("An amp motor failed to config", AlertType.ERROR);

  private TalonFXConfiguration configs = new TalonFXConfiguration();

  public final TalonFX motor1 = new TalonFX(IDs.ROLLER_MOTOR);

  private final TalonSRX sensorTalon = new TalonSRX(42);

  private Alert encoderDisconnected = new Alert("Arm encoder is disconnected", AlertType.ERROR);
  private Alert falconFailed = new Alert("Arm TalonFX failed to configure, possibly disconnected", AlertType.ERROR);

  private static final Map<AmpStates, Double> ANGLES = Map.ofEntries(
        entry(AmpStates.TRANSFER, 100.0),
        entry(AmpStates.SHOOT, 18.0),
        entry(AmpStates.STOW, 140.0),
        entry(AmpStates.UNLOAD, 70.0));

  /** Creates a new ExampleSubsystem. */
  public AmpSubsystem() {
    angleMotor = new CANSparkMax(IDs.ANGLE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

    // Reset each motor so the configs are known-good
    if (angleMotor.restoreFactoryDefaults() != REVLibError.kOk) {
      motorFailed.set(true);
    }  

    angleMotor.setInverted(false);

    angleMotor.setSmartCurrentLimit(40);

    angleMotor.setIdleMode(IdleMode.kBrake);

    angle_pidController = angleMotor.getPIDController();

    angle_encoder = angleMotor.getEncoder();

    angle_encoder.setPositionConversionFactor(14.0);

    kP = OuttakeGainsAmp.kP; 
    kI = OuttakeGainsAmp.kI;
    kD = OuttakeGainsAmp.kD;
    // SmartDashboard.putNumber("shooterKp", kP);
    // SmartDashboard.putNumber("shooterKi", kI);
    // SmartDashboard.putNumber("shooterKd", kD);
    kIz = OuttakeGainsAmp.kIz; 
    kFF = OuttakeGainsAmp.kFF; 
    kMaxOutput = OuttakeGainsAmp.kMaxOutput; 
    kMinOutput = OuttakeGainsAmp.kMinOutput;
    maxRPM = OuttakeGainsAmp.maxRPM;

    angle_pidController.setP(kP);
    angle_pidController.setI(kI);
    angle_pidController.setD(kD);
    angle_pidController.setIZone(kIz);
    angle_pidController.setFF(kFF);
    angle_pidController.setOutputRange(kMinOutput, kMaxOutput);

    motor1.setNeutralMode(NeutralModeValue.Coast);

    motor1.set(0.0);

    sensorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    sensorTalon.configFeedbackNotContinuous(true, 0);
    angle_encoder.setPosition(Math.floorMod((long)(((sensorTalon.getSelectedSensorPosition()) * 360 / 4096)+118), (long)360.0));
  }

  public Command runPIDCommand() {
    return runOnce(() -> {
        angle_pidController.setReference(this.setpoint, CANSparkBase.ControlType.kPosition);
    });
  }
  
  public Command runPIDCommand(AmpStates angle) {
    return runOnce(() -> {
      this.setpoint = ANGLES.get(angle);
      angle_pidController.setReference(this.setpoint, CANSparkBase.ControlType.kPosition);
    });
  }

  public Command runPID(AmpStates angle) {
    return runOnce(() -> {
      this.setpoint = ANGLES.get(angle);
      angle_pidController.setReference(this.setpoint, CANSparkBase.ControlType.kPosition);
    });
  }

  public Command increaseAngle() {
    return runOnce(() -> {
      angle_pidController.setReference(++this.setpoint, CANSparkBase.ControlType.kPosition);
    });
    }

  public Command setAngle(AmpStates angle) {
    return runOnce(() -> {
      this.setpoint = ANGLES.get(angle); 
    });
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("amp_angle", angle_encoder.getPosition());
    SmartDashboard.putNumber("amp_angle_setpoint", this.setpoint);
    SmartDashboard.putNumber("amp_encoder", sensorTalon.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public enum AmpStates {
    TRANSFER,
    /** Lowered for intaking */
    SHOOT,
    /** Stowed - entirely within frame */
    STOW,
    /** Raised to shoot in speaker */
    UNLOAD
}
}
