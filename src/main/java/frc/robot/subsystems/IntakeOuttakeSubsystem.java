// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeOuttakeSubsystem extends SubsystemBase {
  private CANSparkBase intakeMotor;
  private CANSparkBase outtakeTopMotor;
  private CANSparkBase outtakeBottomMotor;
  private SparkPIDController top_pidController;
  private SparkPIDController bottom_pidController;
  private RelativeEncoder top_encoder;
  private RelativeEncoder bottom_encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public DigitalInput limitSwitch = new DigitalInput(9);

  /** Creates a new ExampleSubsystem. */
  public IntakeOuttakeSubsystem() {
    intakeMotor = new CANSparkMax(IDs.INTAKE_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    outtakeTopMotor = new CANSparkMax(IDs.OUTTAKE_TOP_MOTOR, CANSparkLowLevel.MotorType.kBrushless);
    outtakeBottomMotor = new CANSparkMax(IDs.OUTTAKE_BOTTOM_MOTOR, CANSparkLowLevel.MotorType.kBrushless);

    // Reset each motor so the configs are known-good
    intakeMotor.restoreFactoryDefaults();
    outtakeTopMotor.restoreFactoryDefaults();
    outtakeBottomMotor.restoreFactoryDefaults();

    intakeMotor.setInverted(false);

    intakeMotor.setSmartCurrentLimit(80);
    outtakeTopMotor.setSmartCurrentLimit(80);
    outtakeBottomMotor.setSmartCurrentLimit(80);

    intakeMotor.setIdleMode(IdleMode.kBrake);
    outtakeTopMotor.setIdleMode(IdleMode.kCoast);
    outtakeBottomMotor.setIdleMode(IdleMode.kCoast);

    top_pidController = outtakeTopMotor.getPIDController();
    bottom_pidController = outtakeBottomMotor.getPIDController();

    top_encoder = outtakeTopMotor.getEncoder();
    bottom_encoder = outtakeBottomMotor.getEncoder();

    kP = 1e-5; 
    kI = 1e-7;
    kD = 1e-4;
    SmartDashboard.putNumber("shooterKp", kP);
    SmartDashboard.putNumber("shooterKi", kI);
    SmartDashboard.putNumber("shooterKd", kD);
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    top_pidController.setP(kP);
    top_pidController.setI(kI);
    top_pidController.setD(kD);
    top_pidController.setIZone(kIz);
    top_pidController.setFF(kFF);
    top_pidController.setOutputRange(kMinOutput, kMaxOutput);

    bottom_pidController.setP(kP);
    bottom_pidController.setI(kI);
    bottom_pidController.setD(kD);
    bottom_pidController.setIZone(kIz);
    bottom_pidController.setFF(kFF);
    bottom_pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("shooterRPM", 1000.0);
  }

  public boolean intakeIsStalled() {
    return !limitSwitch.get();
  }
  
  public double getShooterRPM(double requestedSpeed) {
    return SmartDashboard.getNumber("shooterRPM", 1000.0); // we need to test what the note speed is for a givin rpm
  }

  public void setMotors(double intake, double outtake) {
    intakeMotor.set(intake);
    SmartDashboard.putNumber("shooterSpeedSetpoint", outtake);
    top_pidController.setReference(-outtake, CANSparkMax.ControlType.kVelocity);
    bottom_pidController.setReference(outtake, CANSparkMax.ControlType.kVelocity);
  }

  public Command setMotors(double intake, DoubleSupplier outtake) {
    return runOnce(() -> {
      intakeMotor.set(intake);
      SmartDashboard.putNumber("shooterSpeedSetpoint", outtake.getAsDouble());
      top_pidController.setReference(-outtake.getAsDouble(), CANSparkMax.ControlType.kVelocity);
      bottom_pidController.setReference(outtake.getAsDouble(), CANSparkMax.ControlType.kVelocity);
    });
  }

  public Command startOutake() {
    return runOnce(() -> {
      this.setMotors(0, Constants.Speeds.OUTTAKE_SPEED);
    });
  }

  public Command stopCommand() {
    return runOnce(() -> {
      this.setMotors(0, 0);
    });
  }

  public Command ejectCommand() {
    return runOnce(() -> {
      this.setMotors(-Constants.Speeds.INTAKE_SPEED, 0);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeMotorCurrent", intakeMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooterSpeed", top_encoder.getVelocity());
    kP = SmartDashboard.getNumber("shooterKp", 0);
    kI = SmartDashboard.getNumber("shooterKi", 0);
    kD = SmartDashboard.getNumber("shooterKd", 0);
    top_pidController.setP(kP);
    top_pidController.setI(kI);
    top_pidController.setD(kD);
    bottom_pidController.setP(kP);
    bottom_pidController.setI(kI);
    bottom_pidController.setD(kD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
