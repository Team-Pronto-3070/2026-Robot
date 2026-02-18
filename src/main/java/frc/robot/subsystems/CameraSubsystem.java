package frc.robot.subsystems;

import java.lang.constant.Constable;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private final Transform3d cameraTransform;
    private final Matrix<N3, N1> kSingleTagStdDevs;
    private final Matrix<N3, N1> kMultiTagStdDevs;

    private final PhotonPoseEstimator poseEstimator;

    private Optional<EstimatedRobotPose> latestEstimation = Optional.empty();

    private Matrix<N3, N1> curStdDevs;

    public CameraSubsystem(Constants.Vision.CameraParams params) {
        // if (!RobotBase.isReal()) { // if in the simulator
        // camera = null;
        // cameraTransform = null;
        // poseEstimator = null;

        // return;
        // }

        camera = new PhotonCamera(params.name);
        cameraTransform = params.transform;
        kSingleTagStdDevs = params.singleTagStdDevs;
        kMultiTagStdDevs = params.multiTagStdDevs;

        poseEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), cameraTransform);
    }

    @Override
    public void periodic() {
        // if (camera == null) {
        // return;
        // }

        latestEstimation = Optional.empty();

        var result = camera.getAllUnreadResults();

        if (!result.isEmpty()) {
            var latest = result.get(result.size() - 1); // Items are ordered oldest to newest

            var optional = poseEstimator.estimateCoprocMultiTagPose(latest);

            // If not present and we try to use it it will crash code
            if (optional.isPresent()) {
                // Use the multi tag pose if available
                latestEstimation = optional;

            } else {
                // Otherwise, fall back to lowest ambiguity single tag pose
                optional = poseEstimator.estimateLowestAmbiguityPose(latest);

                if (optional.isPresent()) {
                    latestEstimation = optional;
                }
            }

            updateEstimationStdDevs(optional, latest.getTargets());
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public Optional<EstimatedRobotPose> getLatestEstimation() {
        return latestEstimation;
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}