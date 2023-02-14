package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.*;
import frc.robot.commands.swerve.BalanceThing;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import java.io.File;
import java.nio.file.Files;
import org.photonvision.PhotonPoseEstimator;

public class RobotContainer {
  /* Controllers */
  private final int driverPort = 0;
  private boolean driverDualshock = false;
  private final int operatorPort = 1;
  private boolean operatorDualshock = false;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Arm s_Arm = new Arm();
  private final Elevator s_Elevator = new Elevator();
  private final Vision s_Vision = new Vision();
  public static PhotonPoseEstimator photonPoseEstimator;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    photonPoseEstimator = s_Vision.photonPoseEstimator; // TODO: find a better way to integrate this
    configureButtons();
    putInfoInDashboard();
  }

  private void putInfoInDashboard() {
    var deployDir = Filesystem.getDeployDirectory();
    var deployInfo = "Unable to find, for some reason";
    try {
      deployInfo = Files.readString(new File(deployDir, "info.txt").toPath());
    } catch (Exception e) {
    }
    SmartDashboard.putString("Code Last Deployed", deployInfo);
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
            driver.leftBumper()::getAsBoolean));

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
            driver.L1()::getAsBoolean));

    driver.triangle().onTrue(new InstantCommand(s_Swerve::zeroGyro));
    driver.circle().onTrue(new BalanceThing(s_Swerve));
  }

  private void xboxOperatorConfiguration() {
    final var operator = new CommandXboxController(operatorPort);
    operator.back().onTrue(s_Intake.pistonsConeCmd());
    operator.start().onTrue(s_Intake.pistonsCubeCmd());
    operator.a().whileTrue(s_Intake.spinInCmd());
    operator.x().whileTrue(s_Intake.spinEjectCmd());
    operator.leftBumper().whileTrue(s_Elevator.moveCmd(Constants.Elevator.retractSpeed));
    operator.rightBumper().whileTrue(s_Elevator.moveCmd(Constants.Elevator.extendSpeed));
    s_Arm.setDefaultCommand(
        s_Arm.moveCmd(
            () ->
                Constants.Arm.backSpeed * operator.getLeftTriggerAxis()
                    + Constants.Arm.frontSpeed * operator.getRightTriggerAxis()));
  }

  private void dualshockOperatorConfiguration() {
    final var operator = new CommandPS4Controller(operatorPort);
    operator.share().onTrue(s_Intake.pistonsConeCmd());
    operator.options().onTrue(s_Intake.pistonsCubeCmd());
    operator.cross().whileTrue(s_Intake.spinInCmd());
    operator.square().whileTrue(s_Intake.spinEjectCmd());
    operator.L1().whileTrue(s_Elevator.moveCmd(Constants.Elevator.retractSpeed));
    operator.R1().whileTrue(s_Elevator.moveCmd(Constants.Elevator.extendSpeed));
    s_Arm.setDefaultCommand(
        s_Arm.moveCmd(
            () ->
                Constants.Arm.backSpeed * operator.getL2Axis()
                    + Constants.Arm.frontSpeed * operator.getR2Axis()));
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

  // Command goToCenterCmd = new GoToPoint(s_Swerve, s_Swerve.getPose(), new Pose2d(5, 5, new
  // Rotation2d(0)));
  // goToCenter.onTrue(goToCenterCmd);
  // if (Constants.enablePS5)
  //    goToCenterPS5.onTrue(goToCenterCmd);
}
