package com.team9470.subsystems.vision;

import com.ctre.phoenix6.Utils;

import com.team9470.FieldConstants;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.VisionCameraSnapshot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;

public class VisionDevice {

    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static final AprilTagFieldLayout aprilTagFieldLayout = FieldConstants.defaultAprilTagType.getLayout();
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    public VisionDevice(String name, Transform3d transform) {
        this.photonCamera = new PhotonCamera(name);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, transform);
    }

    // ONLY CALL ONCE PER CAMERA PER ROBOT LOOP
    private int heartbeat = 0;

    public void updatePosition(Swerve swerve) {
        int heartbeatValue = heartbeat++;

        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        telemetry.publishVisionCameraState(
                photonCamera.getName(),
                new VisionCameraSnapshot(photonCamera.isConnected(), heartbeatValue, results.size(), 0, 0.0, 0.0));

        photonPoseEstimator.addHeadingData(Timer.getFPGATimestamp(), swerve.getPose().getRotation());
        for (PhotonPipelineResult result : results) {
            Optional<EstimatedRobotPose> posEstimate = estimateRobotPose(result);
            if (posEstimate.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimatedPose = posEstimate.get();
            Pose3d robotPose = estimatedPose.estimatedPose;
            Pose3d cameraPose = robotPose.plus(photonPoseEstimator.getRobotToCameraTransform());
            double timestamp = Utils.fpgaToCurrentTime(estimatedPose.timestampSeconds);

            if (!hasUsableTargets(result.getTargets())) {
                continue;
            }

            double std_dev_multiplier = 1.0;

            List<Pose3d> tagPoses = new ArrayList<>();

            // Add all tag poses to the list
            for (PhotonTrackedTarget target : result.getTargets()) {
                int tagId = target.getFiducialId();
                // Optional<Pose3d> tagPose = FieldLayout.kTagMap.getTagPose(tagId);
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);
                tagPose.ifPresent(tagPoses::add);
            }

            if (tagPoses.isEmpty())
                continue;

            // Calculate distances and statistics
            Pair<Double, Double> distanceStats = calculateDistanceStatistics(tagPoses, cameraPose);
            double lowestDist = distanceStats.getFirst();
            double avgDist = distanceStats.getSecond();

            // Estimate standard deviation of vision measurement
            double xyStdDev = calculateXYStandardDeviation(lowestDist, avgDist, tagPoses.size(), std_dev_multiplier);

            // Log vision data
            logVisionData(tagPoses, xyStdDev, cameraPose, robotPose, timestamp, results.size(), heartbeatValue);

            if (Vision.getInstance().isVisionDisabled()) {
                continue;
            }
            // Update robot state with vision data

            swerve.addVisionMeasurement(robotPose.toPose2d(), timestamp,
                    new Matrix<>(Nat.N3(), Nat.N1(), new double[] { xyStdDev, xyStdDev, xyStdDev }));
        }
    }

    private Optional<EstimatedRobotPose> estimateRobotPose(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> coprocessorEstimate = photonPoseEstimator.estimateCoprocMultiTagPose(result);
        if (coprocessorEstimate.isPresent()) {
            return coprocessorEstimate;
        }
        return photonPoseEstimator.estimateLowestAmbiguityPose(result);
    }

    static boolean hasUsableTargets(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return false;
        }

        if (targets.size() < 2) {
            return targets.get(0).getPoseAmbiguity() <= 0.2;
        }

        return true;
    }

    private Pair<Double, Double> calculateDistanceStatistics(List<Pose3d> tagPoses, Pose3d cameraPose) {
        double totalTagDist = 0.0;
        double lowestDist = Double.POSITIVE_INFINITY;

        for (Pose3d pose3d : tagPoses) {
            double dist = pose3d.getTranslation().getDistance(cameraPose.getTranslation());
            totalTagDist += dist;
            lowestDist = Math.min(dist, lowestDist);
        }

        double avgDist = totalTagDist / tagPoses.size();
        return new Pair<>(lowestDist, avgDist);
    }

    private double calculateXYStandardDeviation(double lowestDist, double avgDist, int tagCount, double multiplier) {
        double xyStdDev = multiplier
                * (0.1)
                * ((0.01 * Math.pow(lowestDist, 2.0)) + (0.005 * Math.pow(avgDist, 2.0)))
                / tagCount;
        return Math.max(0.02, xyStdDev);
    }

    private void logVisionData(List<Pose3d> tagPoses, double xyStdDev, Pose3d camera_pose, Pose3d robotPose,
            double timestamp,
            int resultCount,
            int heartbeatValue) {

        telemetry.publishVisionCameraState(
                photonCamera.getName(),
                new VisionCameraSnapshot(
                        photonCamera.isConnected(),
                        heartbeatValue,
                        resultCount,
                        tagPoses.size(),
                        xyStdDev,
                        timestamp));
        telemetry.publishVisionCameraGeometry(
                photonCamera.getName(),
                tagPoses.toArray(new Pose3d[0]),
                camera_pose,
                robotPose,
                Swerve.getInstance().getPose());
    }

    /**
     * Get the Photon Camera object
     * 
     * @return The Photon Camera object
     */
    public PhotonCamera returnCam() {
        return photonCamera;
    }

    /**
     * Get the Photon Pose Estimator object
     * 
     * @return The Photon Pose Estimator object
     */
    public PhotonPoseEstimator returnPoseEstimator() {
        return photonPoseEstimator;
    }

    public boolean isConnected() {
        return photonCamera.isConnected();
    }
}
