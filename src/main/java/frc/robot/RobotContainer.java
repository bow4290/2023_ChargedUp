package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driverPS5 = new Joystick(1);

    /* Drive Controls */
    private final int forwardBackwardAxis = XboxController.Axis.kLeftY.value;
    private final int leftRightAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int forwardBackwardAxisPS5 = PS4Controller.Axis.kLeftY.value;
    private final int leftRightAxisPS5 = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxisPS5 = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton zeroGyroPS5 = new JoystickButton(driverPS5, PS4Controller.Button.kTriangle.value);

    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton robotCentricPS5 = new JoystickButton(driverPS5, PS4Controller.Button.kL1.value);

    private final JoystickButton resetPose = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton resetPosePS5 = new JoystickButton(driverPS5, PS4Controller.Button.kSquare.value);

    private final JoystickButton goToCenter = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton goToCenterPS5 = new JoystickButton(driverPS5, PS4Controller.Button.kShare.value);

    private final JoystickButton tryToBalance = new JoystickButton(driver, XboxController.Button.kB.value);
    // todo figure out what the ps5 equivalent of B is?
    //private final JoystickButton tryToBalancePS5 = new JoystickButton(driverPS5, PS4Controller.Button.kc.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private AprilTagFieldLayout aprilLayout;
    public static PhotonCamera cam;
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    // TODO: update
    public static PhotonPoseEstimator photonPoseEstimator;


    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(forwardBackwardAxis) - 
                    (Constants.enablePS5 ? driverPS5.getRawAxis(forwardBackwardAxisPS5) : 0), 
                () -> -driver.getRawAxis(leftRightAxis) - 
                    (Constants.enablePS5 ? driverPS5.getRawAxis(leftRightAxisPS5) : 0), 
                () -> -driver.getRawAxis(rotationAxis) - 
                    (Constants.enablePS5 ? driverPS5.getRawAxis(rotationAxisPS5) : 0), 
                () -> robotCentric.getAsBoolean() || 
                    (Constants.enablePS5 ? robotCentricPS5.getAsBoolean() : false)
            )
        );
        try {aprilLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);} catch (Exception e) {};
        cam = new PhotonCamera(Constants.Limelight.camName);
        cam.setLED(VisionLEDMode.kBlink);
        photonPoseEstimator = new PhotonPoseEstimator(aprilLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, Constants.Limelight.robotToCam);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));
        if (Constants.enablePS5)
            zeroGyroPS5.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));

        InstantCommand resetPoseCmd = new InstantCommand(() -> {
          photonPoseEstimator.setReferencePose(s_Swerve.getPose());
          Optional<EstimatedRobotPose> res = photonPoseEstimator.update();
          if (res.isPresent()) {
            EstimatedRobotPose camPose = res.get();
            s_Swerve.resetOdometry(camPose.estimatedPose.toPose2d());
          }
        }, s_Swerve);
        resetPose.onTrue(resetPoseCmd);
        if (Constants.enablePS5)
            resetPosePS5.onTrue(resetPoseCmd);

        Command goToCenterCmd = new GoToPoint(s_Swerve, s_Swerve.getPose(), new Pose2d(5, 5, new Rotation2d(0)));
        goToCenter.onTrue(goToCenterCmd);
        if (Constants.enablePS5)
            goToCenterPS5.onTrue(goToCenterCmd);

        tryToBalance.whileTrue(new BalanceThing(s_Swerve));
        // TODO: PS5 balance button
    }

    public Command getAutonomousCommand() {
        return new exampleAuto(s_Swerve);
    }
}
