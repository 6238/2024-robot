// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AmpSubsystem.AmpStates;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class TransferP1Command extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeOuttakeSubsystem intake;
  private final AmpSubsystem amp;
  private double position;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TransferP1Command(IntakeOuttakeSubsystem subsystem1, AmpSubsystem subsystem2) {
    intake = subsystem1;
    amp = subsystem2;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem1, subsystem2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = amp.motor1.getPosition().getValue();
    amp.motor1.setNeutralMode(NeutralModeValue.Brake);
    intake.setMotors(Constants.Speeds.INTAKE_SPEED, Constants.Speeds.SHOOTER_TRANSFER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotors(0.0,0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return amp.motor1.getPosition().getValue() < position - .1;
  }
}
