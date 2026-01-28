package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera frontCamera = new PhotonCamera("Front");

    // 12.5 inches is 0.3175 meters
    // private final Transform3d frontCameraTransform = new Transform3d(new
    // Translation3d(0, 0.3175, 0),
    // new Rotation3d(0, 30, 0));
    private final Transform3d frontCameraTransform = new Transform3d(
            // new Translation3d(
            //         Distance.ofBaseUnits(0, Inches),
            //         Distance.ofBaseUnits(7.5, Inches),
            //         Distance.ofBaseUnits(0, Inches)),
            new Translation3d(0, 0.19, 0),
            new Rotation3d(0, 0.52, 0));

    private final PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), frontCameraTransform);

    private EstimatedRobotPose visionEstimation = null;

    public VisionSubsystem() {

    }

    @Override
    public void periodic() {
        var result = frontCamera.getAllUnreadResults();

        // System.out.println("Vision Periodic");

        if (!result.isEmpty()) {
            // System.out.println("Vision has result");
            var latest = result.get(result.size() - 1);

            if (latest.hasTargets()) {

                var optional = poseEstimator.estimateCoprocMultiTagPose(latest);

                if (optional.isPresent()) {
                    visionEstimation = optional.get();
                } else {
                    optional = poseEstimator.estimateLowestAmbiguityPose(latest);

                    if (optional.isPresent()) {
                        visionEstimation = optional.get();
                    }
                }
            }
        }
    }

    @Override
    public void simulationPeriodic() {

    }

    public EstimatedRobotPose getEstimation() {
        return visionEstimation;
    }
}
