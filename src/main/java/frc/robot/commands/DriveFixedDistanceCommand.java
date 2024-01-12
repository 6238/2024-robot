// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveFixedDistanceCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private double distance;
  private double heading;
  private double speed;
  private Timer timer;

  /**
   * Command that drives a fixed distance.
   * Units are meters, seconds, and degrees.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveFixedDistanceCommand(SwerveSubsystem subsystem, double distance, double heading, double speed) {
    swerveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    this.distance = distance;
    this.heading = Units.degreesToRadians(heading);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
    Translation2d translation = new Translation2d(speed * Math.cos(heading), speed * Math.sin(heading));
    swerveSubsystem.drive(translation, 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds()); // STOP
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(distance / speed);
  }
}
