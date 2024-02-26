// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
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

    outtakeBottomMotor.follow(outtakeTopMotor, true);
  }



  // public static Double lastspeed = null;
  // public boolean intakeIsStalled() {
  //   double current_speed = Math.abs(intakeMotor.getEncoder().getVelocity());
  //   if (lastspeed == null){
  //     lastspeed = current_speed;
  //     return false;
  //   }
  //   if (((current_speed - lastspeed) < Constants.MIN_DELTA_V) && (current_speed < Constants.MIN_V)){
  //     lastspeed = null;
  //     return true;
  //   }
  //   else{
  //     lastspeed = current_speed;
  //     return false;
  //   }
  // }

  public boolean intakeIsStalled() {
    return !limitSwitch.get();
  }

  public void setMotors(double intake, double outtake) {
    intakeMotor.set(intake);
    outtakeTopMotor.set(outtake);
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
    SmartDashboard.putNumber("current", intakeMotor.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
