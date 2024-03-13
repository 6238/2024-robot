// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class AutoAimCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem arm;
  private final SwerveSubsystem swerve;
  private final IntakeOuttakeSubsystem shooter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAimCommand(ArmSubsystem subsystem, SwerveSubsystem swerveSubsystem, IntakeOuttakeSubsystem shooterSubsystem) {
    arm = subsystem;
    swerve = swerveSubsystem;
    shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, swerveSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterAngle_when_down = Math.toRadians(67.94);

    double g = 9.81;
    double shooter_hight = 0.0;
    Pose2d pose = swerve.getPose();
    double x = pose.getX();
    double y = pose.getY();
    double noteVelocity = 7.0; // this is just a placeholder for testing, we need to actualy test what speed our shooter shoots and determine the best value experimentaly.
    double speakerX = 0.0; // currently just the blue speaker
    double speakerY = 5.547868;
    double speakerZ = 1.451102;
    ChassisSpeeds velocity = swerve.getRobotVelocity();
    double robotVx = velocity.vxMetersPerSecond;
    double robotVy = velocity.vyMetersPerSecond;

    double birdsEyeDistance = Math.sqrt(Math.pow((speakerX - x), 2) + Math.pow((speakerY - y), 2));

    double horizontalSpeed = birdsEyeDistance * Math.sqrt(Math.sqrt(2 * g * Math.pow(noteVelocity, 2) * (speakerZ - shooter_hight) - Math.pow((g * birdsEyeDistance), 2) + Math.pow(noteVelocity, 4)) + g * (speakerZ - shooter_hight) + Math.pow(noteVelocity, 2)) 
    / (Math.sqrt(2) * Math.sqrt(Math.pow((speakerZ - shooter_hight), 2) + Math.pow(birdsEyeDistance, 2)));
    
    double vx = (horizontalSpeed * (speakerX - x)) / birdsEyeDistance;
    double ySpeed = Math.sqrt(Math.pow(horizontalSpeed, 2) - Math.pow(vx, 2));
    double vy = y < speakerY ? ySpeed : -ySpeed;

    double shooterVx = vx - robotVx;
    double shooterVy = vy - robotVy;
    double shooterVz = Math.sqrt(Math.pow(noteVelocity, 2) - Math.pow(horizontalSpeed, 2));

    double shooterPitch = Math.atan(shooterVz / Math.sqrt(Math.pow(shooterVx, 2) + Math.pow(shooterVy, 2)));
    double robotYaw = Math.atan(shooterVx / shooterVy);

    SmartDashboard.putNumber("robotYaw", robotYaw);
    SmartDashboard.putBoolean("angleControl", true);

    double armAngle = shooterAngle_when_down - shooterPitch;

    arm.setAngle(armAngle);

    double shooterSpeed = Math.sqrt(Math.pow(shooterVx, 2) + Math.pow(shooterVy, 2) + Math.pow(shooterVz, 2));

    shooter.setMotors(0.0, shooter.getShooterRPM(shooterSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("angleControl", false);
  }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  // }
}
