package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    private final Transform3d cameraTransform;

    private final PhotonPoseEstimator poseEstimator;

    private EstimatedRobotPose visionEstimation = new EstimatedRobotPose(new Pose3d(), 0,
            new ArrayList<PhotonTrackedTarget>());

    public CameraSubsystem(String name, Transform3d transform) {
        camera = new PhotonCamera(name);
        cameraTransform = transform;

        poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), cameraTransform);
    }

    @Override
    public void periodic() {
        var result = camera.getAllUnreadResults();

        if (!result.isEmpty()) {
            var latest = result.get(result.size() - 1); // Items are ordered oldest to newest

            var optional = poseEstimator.estimateCoprocMultiTagPose(latest);

            // If not present and we try to use it it will crash code
            if (optional.isPresent()) {
                // Use the multi tag pose if available
                visionEstimation = optional.get();
            } else {
                // Otherwise, fall back to lowest ambiguity single tag pose
                optional = poseEstimator.estimateLowestAmbiguityPose(latest);

                if (optional.isPresent()) {
                    visionEstimation = optional.get();
                }
            }
        }
    }

    // Get the latest vision-based pose estimation
    public EstimatedRobotPose getEstimation() {
        return visionEstimation;
    }

    // Get the latest robot Pose2d estimation
    public Pose2d getEstimatedPose() {
        return visionEstimation.estimatedPose.toPose2d();
    }

    // Get the timestamp of the latest vision-based pose estimation
    public double getEstimatedTimestamp() {
        return visionEstimation.timestampSeconds;
    }
}