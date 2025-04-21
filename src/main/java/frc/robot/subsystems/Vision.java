package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.Optional;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final PoseEstimator poseEstimator;

    public Vision(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

    camera = new PhotonCamera("USB_Camera");

    AprilTagFieldLayout fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    photonEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        Constants.VisionConstants.ROBOT_TO_CAMERA
    );
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        SmartDashboard.putBoolean("Has Vision Target", result.hasTargets());

        if (result.hasTargets() && poseEstimator.readyToUpdateVision()) {
            Optional<EstimatedRobotPose> estimate = photonEstimator.update(result);
            if (estimate.isPresent()) {
                Pose2d estimatedPose = estimate.get().estimatedPose.toPose2d();
                double timestamp = estimate.get().timestampSeconds;
                poseEstimator.updateVision(estimatedPose, timestamp);

                SmartDashboard.putNumber("Vision X", estimatedPose.getX());
                SmartDashboard.putNumber("Vision Y", estimatedPose.getY());
                SmartDashboard.putNumber("Vision Rotation", estimatedPose.getRotation().getDegrees());

                Logger.recordOutput("Vision Estimated Pose", estimatedPose);
            }
        }
    }

    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    public Rotation2d getTargetYaw() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return Rotation2d.fromDegrees(result.getBestTarget().getYaw());
        }
        return new Rotation2d();
    }

    public PhotonCamera getCamera() {
        return camera;
    }
}
