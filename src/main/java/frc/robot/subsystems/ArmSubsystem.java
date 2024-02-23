// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  // Initialize control requests
  // Position 0, at rest, no FOC, no FF, use config slot 0, always brake when in deadband, no limiting
  private final PositionVoltage voltagePosition = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
  // Brake request to set when neutral

  private double kP = 0.3;
  private double kI = 0;
  private double kD = 0;
  private double setpoint = 0;

  private TalonFXConfiguration configs = new TalonFXConfiguration();


  private final TalonFX motor1 = new TalonFX(11);
  private final TalonFX motor2 = new TalonFX(12);
  private final TalonFX motor3 = new TalonFX(13);

  private final TalonSRX sensorTalon = new TalonSRX(14);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {

    double motorToArm = 17.5; // motor turns needed for the arm to turn fully
    
    configs.Feedback.SensorToMechanismRatio = motorToArm;

    configs.Slot0.kP = kP;
    configs.Slot0.kI = kI;
    configs.Slot0.kD = kD;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("setpoint", setpoint);

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    // Peak output of 130 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 130;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 130;
    
    // Set motors 2 and 3 to follow motor 1
    motor2.setControl(new Follower(motor1.getDeviceID(), false));
    motor2.setNeutralMode(NeutralModeValue.Brake);
    motor3.setControl(new Follower(motor1.getDeviceID(), false));
    motor3.setNeutralMode(NeutralModeValue.Brake);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor1.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // Set wherever we are on init to 0
    motor1.setPosition(0);

    sensorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
  }


  public Command runPIDCommand() {
    return runOnce(() -> {
      motor1.setControl(voltagePosition.withPosition(setpoint));
    });
  }

  public Command increaseSetpointCommand() {
      return runOnce(() -> {
      this.setpoint += 1; // one more motor turn
    });
  }


  public Command decreaseSetpointCommand() {
      return Commands.runOnce(() -> {
      this.setpoint -= 1; // one more motor turn
    });
  }

  private void setAngle(Measure<Angle> angle) {
    this.setpoint = angle.in(Degrees) / 360;
  }

  public Command setAngleCommand(Measure<Angle> angle) {
    return Commands.runOnce(() -> {
      this.setAngle(angle);
    });
    // don't use this.runOnce because it implicitly requires this, which is not what we want (don't stop the loop to change the setpt)
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Pull PD values from dashboard
    this.kP = SmartDashboard.getNumber("kP", kP);
    this.kI = SmartDashboard.getNumber("kI", kI);
    this.kD = SmartDashboard.getNumber("kD", kD);
    // this.setpoint = SmartDashboard.getNumber("setpoint", setpoint);

    // motor1.setPosition(sensorTalon.getSelectedSensorPosition() * (1.0/4096.0));
    // Update our PID controller
    configs.Slot0.kP = this.kP;
    configs.Slot0.kI = this.kI;
    configs.Slot0.kD = this.kD;
    // motor1.getConfigurator().apply(configs);
    motor1.getConfigurator().apply(configs.Slot0);

    SmartDashboard.putNumberArray("motor current draw", new double[] {motor1.getTorqueCurrent().getValue(), motor2.getTorqueCurrent().getValue(), motor3.getTorqueCurrent().getValue()});

    SmartDashboard.putNumber("sensor value in turns", sensorTalon.getSelectedSensorPosition() * (1.0/4096.0));
    SmartDashboard.putNumber("raw sensor value", sensorTalon.getSelectedSensorPosition());
    SmartDashboard.putNumber("falcon position", motor1.getPosition().getValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}