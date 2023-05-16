package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    getEntry("ledMode").setNumber(1);
    getEntry("camMode").setNumber(1);
  }

  public NetworkTableEntry getEntry(String entryName) {
    return table.getEntry(entryName);
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
    if (getEntry("tv").getInteger(1) == 1) {
      double[] posArray =
          getEntry("botpose_wpi" + DriverStation.getAlliance().name().toLowerCase())
              .getDoubleArray(new double[7]);
      position =
          new PoseEstimate(
              new Pose2d(posArray[0], posArray[1], new Rotation2d(5)),
              Timer.getFPGATimestamp() - (posArray[6] / 1000.0));

      SmartDashboard.putString("Last Pose Estimate", position.estimatedPose.toString());
    }

    return Optional.ofNullable(position);
  }
}
