package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;

/**
 * The Vision class interfaces with the PhotonVision (Limelight) and gets robot positions using
 * AprilTags. This is used by the Swerve subsystem.
 */
public class Vision {
  private AprilTagFieldLayout aprilLayout;
  public static PhotonCamera cam;

  public PhotonPoseEstimator photonPoseEstimator;
  public Timer timeSinceLastVision = new Timer();

  public Vision() {
    timeSinceLastVision.start();

    try {
      aprilLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      // This is unnecessary, since code would error. This is impossible, anyways
      SmartDashboard.putString("WARNING: ERROR LOADING LAYOUT!", "yes");
    }

    cam = new PhotonCamera(Constants.Limelight.camName);
    cam.setLED(VisionLEDMode.kOff);

    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilLayout,
            PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
            cam,
            Constants.Limelight.robotToCam);
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
    Optional<EstimatedRobotPose> estimatedPosition = photonPoseEstimator.update();
    PoseEstimate position = null;

    if (estimatedPosition.isPresent()) {
      EstimatedRobotPose camPose = estimatedPosition.get();
      position = new PoseEstimate(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      timeSinceLastVision.reset();
    }

    return Optional.of(position);
  }

  public double lastVisionTime() {
    return timeSinceLastVision.get();
  }
}
