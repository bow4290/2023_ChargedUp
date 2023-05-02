package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;

/**
 * The Vision class interfaces with the Limelight and gets robot positions using AprilTags. This is
 * used by the Swerve subsystem.
 */
public class Vision {
  NetworkTable table;

  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    for (int i = 1; i < 32; i += 4) {
      setLLDriverCmd()
          .beforeStarting(Commands.waitSeconds(i).andThen(Commands.print("Setting LL driver mode")))
          .schedule();
    }
  }

  public void setLLDriver() {
    table.getEntry("ledMode").setNumber(1);
    table.getEntry("camMode").setNumber(1);
  }

  public CommandBase setLLDriverCmd() {
    return Commands.runOnce(this::setLLDriver).ignoringDisable(true);
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
