package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout aprilLayout;
    public static PhotonCamera cam;
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    // TODO: update
    public static PhotonPoseEstimator photonPoseEstimator;
    public Vision() {
        try {aprilLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);} catch (Exception e) {};
        cam = new PhotonCamera(Constants.Limelight.camName);
        cam.setLED(VisionLEDMode.kBlink);
        photonPoseEstimator = new PhotonPoseEstimator(aprilLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, Constants.Limelight.robotToCam);
    }
}