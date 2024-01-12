// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;

import java.util.HashMap;

public final class Autos
{

  /**
   * April Tag field layout.
   */
  private static AprilTagFieldLayout aprilTagField = null;

  private Autos()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command driveAndSpin(SwerveSubsystem swerve)
  {
    return Commands.sequence(
        new RepeatCommand(new InstantCommand(() -> swerve.drive(new Translation2d(1, 0), 5, true), swerve)));
  }

  /**
   * Static factory for a saved path group
   */
  public static Command autoPathGroup(SwerveSubsystem swerve)
  {
    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));

    PIDFConfig drivePID = SwerveParser.pidfPropertiesJson.drive;
    PIDConstants pcDrive = new PIDConstants(drivePID.p, drivePID.i, drivePID.d);
    PIDFConfig anglePID = SwerveParser.pidfPropertiesJson.angle;
    PIDConstants pcAngle = new PIDConstants(anglePID.p, anglePID.i, anglePID.d);

    return Commands.sequence(swerve.createPathPlannerCommand("SamplePath", new PathConstraints(0.25, 1), eventMap, pcDrive, pcAngle, false));
  }

  /**
   * Create a {@link FollowTrajectory} command to go to the April Tag from the current position.
   *
   * @param swerve            Swerve drive subsystem.
   * @param id                April Tag ID to go to.
   * @param rotation          Rotation to go to.
   * @param holonomicRotation Holonomic rotation to be at.
   * @param offset            Offset from the April Tag.
   * @return {@link FollowTrajectory} command. May return null if cannot load field.
   */
  public static Command driveToAprilTag(SwerveSubsystem swerve, int id, Rotation2d rotation,
                                            Rotation2d holonomicRotation, Translation2d offset)
  {
    if (aprilTagField == null) {
      try {
        aprilTagField = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
      } catch (Exception ignored) {
        return null;
      }
    }
    PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4, 3), false,
                                                          PathPoint.fromCurrentHolonomicState(swerve.getPose(),
                                                                                              swerve.getRobotVelocity()),
                                                          new PathPoint(aprilTagField.getTagPose(id).get()
                                                                                     .getTranslation()
                                                                                     .toTranslation2d().plus(offset),
                                                                        rotation, holonomicRotation));
    return Commands.sequence(new FollowTrajectory(swerve, path, false));
  }
}