package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * The Vision class interfaces with the PhotonVision (Limelight) and gets robot positions using
 * AprilTags. This is used by the Swerve subsystem.
 */
public class Vision {
  private AprilTagFieldLayout aprilLayout;

  public Vision() {

    try {
      aprilLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      // This is unnecessary, since code would error. This is impossible, anyways
      SmartDashboard.putString("WARNING: ERROR LOADING LAYOUT!", "yes");
    }
  }

  /** A non-vendor-library-dependent way to hold the estimated robot position */
  public class PoseEstimate {
    public final Pose2d estimatedPose;
    public final double timestampSeconds;

    public PoseEstimate(Pose2d estimatedPose, double timestampSeconds) {
      this.estimatedPose = estimatedPose;
      this.timestampSeconds = timestampSeconds;
    }
  }

  public Optional<PoseEstimate> getEstimatedPose() {
    /*Optional<EstimatedRobotPose> estimatedPosition = photonPoseEstimator.update();
    PoseEstimate position = null;

    if (estimatedPosition.isPresent()) {
      EstimatedRobotPose camPose = estimatedPosition.get();
      position = new PoseEstimate(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      timeSinceLastVision.reset();
    }*/
    PoseEstimate position = null;

    return Optional.ofNullable(position);
  }
}
