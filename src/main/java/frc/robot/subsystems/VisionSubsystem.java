package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private AprilTagFieldLayout layout;
    private PhotonCamera cam;
    private final Transform3d robotToCam = new Transform3d(new Translation3d(0.23, 0, 0), new Rotation3d(0, 0, 0));
    private PhotonPoseEstimator poseEst;
    private SwerveSubsystem swerve;
    private Field2d field = new Field2d();

    /** Create a new subsystem */
    public VisionSubsystem(String camName, SwerveSubsystem swerve) {
        System.out.println("Hello from VisionSubsystem! Initializing using " + camName);
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        cam = new PhotonCamera(camName);

        poseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Camera connected", cam.isConnected());
        if (cam.isConnected()) {
            Optional<EstimatedRobotPose> pose = poseEst.update();
            if (pose.isPresent()) {
                swerve.addVisionPose(pose.get());
                field.setRobotPose(pose.get().estimatedPose.toPose2d());
                SmartDashboard.putData("VisionField", field);
            }
        }

    }

}
