// Copyback (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
@Logged
public class Vision {
    @NotLogged
    PhotonCamera backCam;
    @NotLogged
    PhotonCamera frontCam;
    @NotLogged
    SwerveDrivePoseEstimator odometry;

    PhotonPoseEstimator backEstimator;
    PhotonPoseEstimator frontEstimator;
    PhotonPipelineResult lastResult = new PhotonPipelineResult();

    double backTagDistance = 0;
    double backAmbiguity = 0;
    double backBestID = 0;
    double backOffsetDistance = 0;
    Pose3d backPoseEstimate = new Pose3d();

    double frontTagDistance = 0;
    double frontAmbiguity = 0;
    double frontBestID = 0;
    double frontOffsetDistance = 0;
    Pose3d frontPoseEstimate = new Pose3d();

    @NotLogged
    private static Vision singleton;

    public Vision() {
        odometry = Drivetrain.getInstance().odometry;
        frontCam = new PhotonCamera("frontCam");
        backCam = new PhotonCamera("backCam");

        backEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                backCameraLocation
        );
        frontEstimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark),
                frontCameraLocation
        );
    }

    public static Vision getInstance() {
        if (singleton == null)
            singleton = new Vision();
        return singleton;
    }

    /**
     * wrapper for running all periodic vision code
     */
    public void runVision() {
        for (var result : backCam.getAllUnreadResults()) {
            if (!result.hasTargets())
                continue;
            // if best visible target is too far away for our liking, discard it, else use
            // it
            backTagDistance = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
            if (backTagDistance > maxVisionDistanceTolerance)
                continue;
            // Check if 3D ambiguity is too high and skip if it is. May not be needed
            // because of chosen estimator strategy
            backAmbiguity = result.getBestTarget().poseAmbiguity;
            if (backAmbiguity > maxAmbiguity)
                continue;

            var em = backEstimator.estimateCoprocMultiTagPose(result);
            if (em.isEmpty()) {
                em = backEstimator.estimateLowestAmbiguityPose(result);
            }
            if (em.isEmpty())
                continue;
            backPoseEstimate = em.get().estimatedPose;
            // Check if estimate has us flying in the air and reject
            if (backPoseEstimate.getZ() > zTolerance)
                continue;
            // Check if estimate says we're rolled and reject
            if (backPoseEstimate.getRotation().getX() > rollPitchTolerance)
                continue;
            // Check if estimate says we're pitched and reject
            if (backPoseEstimate.getRotation().getY() > rollPitchTolerance)
                continue;
            // Check if estimate is close enough to where we think we are
            backOffsetDistance = backPoseEstimate.toPose2d().getTranslation()
                    .getDistance(odometry.getEstimatedPosition().getTranslation());
            if (backOffsetDistance > visionPoseDiffTolerance && filterByDistanceFromOdometryPose)
                continue;
            // Measurement passed all filters, add to global pose estimate
            odometry.addVisionMeasurement(backPoseEstimate.toPose2d(), em.get().timestampSeconds);
        }

        for (var result : frontCam.getAllUnreadResults()) {
            if (!result.hasTargets())
                continue;
            // if best visible target is too far away for our liking, discard it, else use
            // it
            frontTagDistance = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
            if (frontTagDistance > maxVisionDistanceTolerance)
                continue;
            // Check if 3D ambiguity is too high and skip if it is. May not be needed
            // because of chosen estimator strategy
            frontAmbiguity = result.getBestTarget().poseAmbiguity;
            if (frontAmbiguity > maxAmbiguity)
                continue;

            var em = frontEstimator.estimateCoprocMultiTagPose(result);
            if (em.isEmpty()) {
                em = frontEstimator.estimateLowestAmbiguityPose(result);
            }
            if (em.isEmpty())
                continue;
            frontPoseEstimate = em.get().estimatedPose;
            // Check if estimate has us flying in the air and reject
            if (frontPoseEstimate.getZ() > zTolerance)
                continue;
            // Check if estimate says we're rolled and reject
            if (frontPoseEstimate.getRotation().getX() > rollPitchTolerance)
                continue;
            // Check if estimate says we're pitched and reject
            if (frontPoseEstimate.getRotation().getY() > rollPitchTolerance)
                continue;
            // Check if estimate is close enough to where we think we are
            frontOffsetDistance = frontPoseEstimate.toPose2d().getTranslation()
                    .getDistance(odometry.getEstimatedPosition().getTranslation());
            if (frontOffsetDistance > visionPoseDiffTolerance && filterByDistanceFromOdometryPose)
                continue;
            // Measurement passed all filters, add to global pose estimate
            odometry.addVisionMeasurement(frontPoseEstimate.toPose2d(), em.get().timestampSeconds);
        }
    }
}
