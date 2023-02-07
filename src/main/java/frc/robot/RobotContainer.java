package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.arm.MovArm;
import frc.robot.commands.elevator.MovElevator;
import frc.robot.commands.intake.PistonTake;
import frc.robot.commands.intake.SpInintake;
import frc.robot.commands.intake.SpInintake.IntakeSpinStatus;
import frc.robot.commands.swerve.BalanceThing;
import frc.robot.commands.swerve.GoToPoint;
import frc.robot.commands.swerve.TeleopSwerve;
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
    private final int driverPort = 0;
    private boolean driverDualshock = true;
    private final int operatorPort = 1;
    private boolean operatorDualshock = true;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Arm s_Arm = new Arm();
    private final Elevator s_Elevator = new Elevator();

    private AprilTagFieldLayout aprilLayout;
    public static PhotonCamera cam;
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    // TODO: update
    public static PhotonPoseEstimator photonPoseEstimator;


    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {

        try {aprilLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);} catch (Exception e) {};
        cam = new PhotonCamera(Constants.Limelight.camName);
        cam.setLED(VisionLEDMode.kBlink);
        photonPoseEstimator = new PhotonPoseEstimator(aprilLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, Constants.Limelight.robotToCam);

        configureButtons();
    }

    private void configureButtons() {
        if (driverDualshock) dualshockDriverConfiguration();
        else xboxDriverConfiguration();
        if (operatorDualshock) dualshockOperatorConfiguration();
        else xboxOperatorConfiguration();
    }

    private void xboxDriverConfiguration() {
        final var driver = new CommandXboxController(driverPort);
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        driver.leftBumper()::getAsBoolean
                )
        );

        driver.y().onTrue(new InstantCommand(s_Swerve::zeroGyro));
        driver.b().onTrue(new BalanceThing(s_Swerve));
    }
    private void dualshockDriverConfiguration() {
        final var driver = new CommandPS4Controller(driverPort);
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        () -> -driver.getRightX(),
                        driver.L1()::getAsBoolean
                )
        );

        driver.triangle().onTrue(new InstantCommand(s_Swerve::zeroGyro));
        driver.circle().onTrue(new BalanceThing(s_Swerve));
    }

    private void xboxOperatorConfiguration() {
        final var operator = new CommandXboxController(operatorPort);
        operator.back().onTrue(new PistonTake(s_Intake, Intake.IntakePistonStatus.Cone));
        operator.start().onTrue(new PistonTake(s_Intake, Intake.IntakePistonStatus.Cube));
        operator.a().whileTrue(new SpInintake(s_Intake, IntakeSpinStatus.Intake));
        operator.x().whileTrue(new SpInintake(s_Intake, IntakeSpinStatus.Eject));
        operator.leftBumper().whileTrue(new MovElevator(s_Elevator, Constants.Elevator.retractSpeed));
        operator.rightBumper().whileTrue(new MovElevator(s_Elevator, Constants.Elevator.extendSpeed));
        s_Arm.setDefaultCommand(
                new MovArm(s_Arm, () -> Constants.Arm.downSpeed * operator.getLeftTriggerAxis() +
                        Constants.Arm.upSpeed * operator.getRightTriggerAxis()
                )
        );
    }
    private void dualshockOperatorConfiguration() {
        final var operator = new CommandPS4Controller(operatorPort);
        operator.share().onTrue(new PistonTake(s_Intake, Intake.IntakePistonStatus.Cone));
        operator.options().onTrue(new PistonTake(s_Intake, Intake.IntakePistonStatus.Cube));
        operator.cross().whileTrue(new SpInintake(s_Intake, IntakeSpinStatus.Intake));
        operator.square().whileTrue(new SpInintake(s_Intake, IntakeSpinStatus.Eject));
        operator.L1().whileTrue(new MovElevator(s_Elevator, Constants.Elevator.retractSpeed));
        operator.R1().whileTrue(new MovElevator(s_Elevator, Constants.Elevator.extendSpeed));
        s_Arm.setDefaultCommand(
                new MovArm(s_Arm, () -> Constants.Arm.downSpeed * operator.getL2Axis() +
                        Constants.Arm.upSpeed * operator.getR2Axis()
                )
        );
    }

    public Command getAutonomousCommand() {
        return new exampleAuto(s_Swerve);
    }


// todo: fix and re-integrate this code
     /*    InstantCommand resetPoseCmd = new InstantCommand(() -> {
          photonPoseEstimator.setReferencePose(s_Swerve.getPose());
          Optional<EstimatedRobotPose> res = photonPoseEstimator.update();
          if (res.isPresent()) {
            EstimatedRobotPose camPose = res.get();
            s_Swerve.resetOdometry(camPose.estimatedPose.toPose2d());
          }
        }, s_Swerve);*/
    //       resetPose.onTrue(resetPoseCmd);
    //      if (Constants.enablePS5)
    //          resetPosePS5.onTrue(resetPoseCmd);

    // Command goToCenterCmd = new GoToPoint(s_Swerve, s_Swerve.getPose(), new Pose2d(5, 5, new Rotation2d(0)));
    // goToCenter.onTrue(goToCenterCmd);
    //if (Constants.enablePS5)
    //    goToCenterPS5.onTrue(goToCenterCmd);
}
