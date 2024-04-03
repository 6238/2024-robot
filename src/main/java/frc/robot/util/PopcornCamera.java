package frc.robot.util;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.telemetry.Alert;

public class PopcornCamera {
    private PhotonCamera cam;
    private PhotonPoseEstimator poseEst;
    
    private Alert camDisconnected;

    public PopcornCamera(String camName, Transform3d robotToCam, AprilTagFieldLayout layout) {
        cam = new PhotonCamera(camName);
        poseEst = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, cam, robotToCam);

        camDisconnected = new Alert("Camera " + " is disconnected!", Alert.AlertType.ERROR);
    }

    public Optional<EstimatedRobotPose> update() {
        if (!cam.isConnected()) {
            camDisconnected.set(true);
            DataLogManager.log("Skipping pose update for camera " + cam.getName() + " due to disconnect!");
            return Optional.empty();
        } else {
            camDisconnected.set(false);
            Optional<EstimatedRobotPose> pose = poseEst.update();
            if (pose.isPresent() && poseValid(pose.get())) {

                if (!DriverStation.isFMSAttached()) {
                    
                }

                return pose;
            } else {
                return Optional.empty();
            }
        }
    }

    private static boolean poseValid(EstimatedRobotPose pose) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        boolean inRange;
        if (ally.isEmpty()) {
            return true;
        } else {
            if (ally.get() == Alliance.Blue) {
                    inRange = pose.estimatedPose.getX() < 3.5;
                } else {
                    inRange = pose.estimatedPose.getX() > 16.55445;
                }
            return inRange;
        }
    }
}
