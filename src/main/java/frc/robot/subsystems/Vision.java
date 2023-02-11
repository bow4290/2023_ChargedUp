package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;

public class Vision extends SubsystemBase {
  private AprilTagFieldLayout aprilLayout;
  public static PhotonCamera cam;
  
  public PhotonPoseEstimator photonPoseEstimator;

  public Vision() {
    try {
      aprilLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
    }
    ;
    cam = new PhotonCamera(Constants.Limelight.camName);
    cam.setLED(VisionLEDMode.kBlink);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilLayout,
            PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            cam,
            Constants.Limelight.robotToCam);
  }
}
