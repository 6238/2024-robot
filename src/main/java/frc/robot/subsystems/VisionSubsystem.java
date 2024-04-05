package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PopcornCamera;

import static edu.wpi.first.units.Units.*;

public class VisionSubsystem extends SubsystemBase {
    private AprilTagFieldLayout layout;
    private final Transform3d robotToCam_A = new Transform3d(
            new Translation3d(Inches.of(-13.5), Inches.of(-13.5), Inches.of(10.5)),
            new Rotation3d(Degrees.of(180).in(Radians), Degrees.of(-55).in(Radians), Degrees.of(210).in(Radians)));
    private final Transform3d robotToCam_B = new Transform3d(
            new Translation3d(Inches.of(-13.5), Inches.of(13.5), Inches.of(10.5)),
            new Rotation3d(Degrees.of(0).in(Radians), Degrees.of(-55).in(Radians), Degrees.of(150).in(Radians)));
    private SwerveSubsystem swerve;
    // private Field2d field = new Field2d();

    private PopcornCamera camA;
    private PopcornCamera camB;

    /** Create a new subsystem */
    public VisionSubsystem(SwerveSubsystem swerve) {
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        camA = new PopcornCamera(Constants.PHOTONCAMERA_NAME, robotToCam_A, layout);
        camB = new PopcornCamera(Constants.PHOTONCAMERA_NAME2, robotToCam_B, layout);

        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> poseA = camA.update();
        if (poseA.isPresent()) {
            swerve.addVisionPose(poseA.get(), Constants.VISION_STDDEV);
        }

        Optional<EstimatedRobotPose> poseB = camB.update();
        if (poseB.isPresent()) {
            swerve.addVisionPose(poseB.get(), Constants.VISION_STDDEV);
        }
    }
}
