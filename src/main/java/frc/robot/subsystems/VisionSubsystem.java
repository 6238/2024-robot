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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.Alert;
import frc.robot.telemetry.Alert.AlertType;

import static edu.wpi.first.units.Units.*;

public class VisionSubsystem extends SubsystemBase {
    private AprilTagFieldLayout layout;
    private PhotonCamera cam_A;
    private final Transform3d robotToCam_A = new Transform3d(
            new Translation3d(Inches.of(-11.25), Inches.of(-4.75), Inches.of(8.5)),
            new Rotation3d(Degrees.of(180).in(Radians), Degrees.of(-55).in(Radians), Degrees.of(150).in(Radians)));
    private PhotonCamera cam_B;
    private final Transform3d robotToCam_B = new Transform3d(
            new Translation3d(Inches.of(-11.25), Inches.of(-4.75), Inches.of(8.5)),
            new Rotation3d(Degrees.of(180).in(Radians), Degrees.of(-55).in(Radians), Degrees.of(210).in(Radians)));
    private PhotonPoseEstimator poseEst;
    private SwerveSubsystem swerve;
    private Field2d field = new Field2d();

    private Alert camDisconnected = new Alert("AprilTag camera is disconnected, pose-est will not work!",
            AlertType.ERROR);

    /** Create a new subsystem */
    public VisionSubsystem(String camName, SwerveSubsystem swerve) {
        DataLogManager.log("Initializing vision using " + camName);
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        cam_A = new PhotonCamera(camName);
        cam_A.setDriverMode(false);

        poseEst = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam_A, robotToCam_A);
        this.swerve = swerve;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Camera connected", cam_A.isConnected());
        if (!cam_A.isConnected()) {
            camDisconnected.set(true);
        }
        if (cam_A.isConnected()) {
            Optional<EstimatedRobotPose> pose = poseEst.update();
            Optional<Alliance> ally = DriverStation.getAlliance();
            boolean inRange;
            if (pose.isPresent()) {
                boolean blue = (ally.get() == Alliance.Blue);

                if (blue) {
                    inRange = pose.get().estimatedPose.getX() < 3.5;
                } else {
                    inRange = pose.get().estimatedPose.getX() > 16.55445;
                }
                if (pose.isPresent() && inRange) {
                    swerve.addVisionPose(pose.get(), Constants.VISION_STDDEV);
                    field.setRobotPose(pose.get().estimatedPose.toPose2d());
                    SmartDashboard.putData("VisionField", field);
                }
            }
        }
    }
}
