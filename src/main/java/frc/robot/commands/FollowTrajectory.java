package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;

public class FollowTrajectory extends SequentialCommandGroup
{

  public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry)
  {
    addRequirements(drivebase);

    if (resetOdometry)
    {
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    }

    PIDFConfig drivePID = SwerveParser.pidfPropertiesJson.drive;
    PIDFConfig anglePID = SwerveParser.pidfPropertiesJson.angle;

    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            drivePID.createPIDController(),
            drivePID.createPIDController(),
            anglePID.createPIDController(),
            drivebase::setChassisSpeeds,
            drivebase)
        );
  }
}