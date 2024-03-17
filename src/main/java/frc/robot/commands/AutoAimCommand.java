// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.telemetry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class AutoAimCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem arm;
  private final IntakeOuttakeSubsystem shooter;
  private final DoubleSupplier poseX;
  private final DoubleSupplier poseY;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAimCommand(ArmSubsystem subsystem, IntakeOuttakeSubsystem shooterSubsystem, DoubleSupplier poseX, DoubleSupplier poseY) {
    arm = subsystem;
    shooter = shooterSubsystem;
    this.poseX = poseX;
    this.poseY = poseY;

    SmartDashboard.putBoolean("angleControl", false);
    SmartDashboard.putNumber("noteVelocity", 10.0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterAngle_when_down = 67.94;

    double g = 9.81;
    double shooter_hight = 0.4; // Note sure if this is actualy constant, but I'm just going to assume so.
    double noteVelocity = SmartDashboard.getNumber("noteVelocity", 10.0); // this is just a placeholder for testing, we need to actualy test what speed our shooter shoots and determine the best value experimentaly.
    double x = poseX.getAsDouble();
    double y = poseY.getAsDouble();
    double speakerX = 0.0; // currently just the blue speaker
    double speakerY = 5.547868;
    double speakerZ = 2.4;
    double[] velocity = SwerveDriveTelemetry.measuredChassisSpeeds;
    double robotVx = velocity[0];
    double robotVy = velocity[1];

    double birdsEyeDistance = Math.sqrt(Math.pow((speakerX - x), 2) + Math.pow((speakerY - y), 2));

    double horizontalSpeed = birdsEyeDistance * Math.sqrt(Math.sqrt(2 * g * Math.pow(noteVelocity, 2) * (speakerZ - shooter_hight) - Math.pow((g * birdsEyeDistance), 2) + Math.pow(noteVelocity, 4)) + g * (speakerZ - shooter_hight) + Math.pow(noteVelocity, 2)) 
    / (Math.sqrt(2) * Math.sqrt(Math.pow((speakerZ - shooter_hight), 2) + Math.pow(birdsEyeDistance, 2)));
    
    double vx = (horizontalSpeed * (speakerX - x)) / birdsEyeDistance;
    double ySpeed = Math.sqrt(Math.pow(horizontalSpeed, 2) - Math.pow(vx, 2));
    double vy = y < speakerY ? ySpeed : -ySpeed;

    double shooterVx = vx - robotVx;
    double shooterVy = vy - robotVy;
    double shooterVz = Math.sqrt(Math.pow(noteVelocity, 2) - Math.pow(horizontalSpeed, 2));

    double shooterPitch = Math.toDegrees(Math.atan(shooterVz / horizontalSpeed));
    double robotYaw =  shooterVy <= 0 ? -Math.atan(shooterVx / shooterVy) + (Math.PI / 2) :  -Math.atan(shooterVx / shooterVy) + (3 * Math.PI / 2);

    SmartDashboard.putNumber("headingSetpoint", robotYaw);
    SmartDashboard.putNumber("headingSetpointDegrees", Math.toDegrees(robotYaw));
    SmartDashboard.putNumber("shooterPitch", shooterPitch);
    
    SmartDashboard.putBoolean("angleControl", true);

    double armAngle = shooterAngle_when_down - shooterPitch;

    arm.setAngle(armAngle);
    arm.runPID();

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
