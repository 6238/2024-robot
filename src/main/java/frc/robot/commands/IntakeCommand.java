// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeOuttakeSubsystem m_subsystem;
  private boolean spinup;
  private boolean beam;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(IntakeOuttakeSubsystem subsystem, boolean spinupBool, boolean beamBreak) {
    m_subsystem = subsystem;
    spinup = spinupBool;
    beam = beamBreak;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (beam) {
      m_subsystem.setMotors(Constants.Speeds.INTAKE_SPEED, spinup ? Constants.Speeds.OUTTAKE_SPEED : 0);
    }
    else {
      m_subsystem.setMotors(Constants.Speeds.INTAKE_SPEED, spinup ? Constants.Speeds.OUTTAKE_SPEED : -100.0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("intake ended" + interrupted);
    m_subsystem.setMotors(0, spinup ? Constants.Speeds.OUTTAKE_SPEED : 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (beam) {
      return m_subsystem.intakeIsStalled();
    // }
    // else {
    //   return Math.abs(m_subsystem.top_encoder.getVelocity()) < 1.0 || m_subsystem.intakeMotor.getOutputCurrent() > 75.0;
    // }
  }
}
