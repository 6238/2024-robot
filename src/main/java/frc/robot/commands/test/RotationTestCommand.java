package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

public class RotationTestCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsystem swerveSubsystem;
  private TestResult testResult;

  public RotationTestCommand(SwerveSubsystem subsystem) {
    swerveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Slowly rotate until gyro angle == 0, then reset pose angle
    System.out.println("INITIALIZING TEST");
    swerveSubsystem.drive(new Translation2d(0, 0), 0.5, false);
    while ((int) swerveSubsystem.getHeading().getDegrees() % 360 != 0) {
      // Do nothing, let it keep spinning
      System.out.println("NOT YET ... at " + swerveSubsystem.getHeading().getDegrees() + " degrees");
    }
    System.out.println("ZEROING, STARTING TEST");
    swerveSubsystem.zeroGyro();
    testResult = TestResult.RUNNING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Slowly rotate while monitoring both pose angle and gyro angle
    // If angles differ by more than 3 degrees, fail!
    if (Math.abs((swerveSubsystem.getHeading().getDegrees() % 360) - (swerveSubsystem.getPose().getRotation().getDegrees() % 360)) > 2.0) {
      testResult = TestResult.FAIL;
      System.out.println("TEST FAILED!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.drive(new ChassisSpeeds()); // STOP
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return true if either command has failed, or if we've reached 360 degrees
    if (swerveSubsystem.getHeading().getDegrees() > 179) {
      testResult = TestResult.PASS;
      System.out.println("TEST PASSED!");
    }
    return testResult != TestResult.RUNNING;
  }
}
