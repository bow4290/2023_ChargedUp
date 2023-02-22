package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private boolean driverDualshock = true;
  private final int operatorPort = 1;
  private boolean operatorDualshock = true;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Arm s_Arm = new Arm();
  private final Elevator s_Elevator = new Elevator();
  private final Vision s_Vision = new Vision();
  public static PhotonPoseEstimator photonPoseEstimator;

  SendableChooser<Command> chooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    photonPoseEstimator = s_Vision.photonPoseEstimator; // TODO: find a better way to integrate this
    configureButtons();
    putInfoInDashboard();

    chooser.setDefaultOption("move forward", new exampleAuto(s_Swerve, s_Arm));
    chooser.addOption("do nothing", new InstantCommand(() -> {}));
    //robot.explode();
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
    driver.b().whileTrue(new BalanceThing(s_Swerve));
    // driver.a().whileTrue(s_Arm.posCmd(0));
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
    driver.circle().whileTrue(new BalanceThing(s_Swerve));
  }

  private void xboxOperatorConfiguration() {
    final var operator = new CommandXboxController(operatorPort);
    operator.back().onTrue(s_Intake.pistonsConeCmd());
    operator.start().onTrue(s_Intake.pistonsCubeCmd());
    operator.a().whileTrue(s_Intake.spinInCmd());
    operator.x().whileTrue(s_Intake.spinEjectCmd());

    // completely out: 42000
    operator.y().whileTrue(s_Elevator.positionCmd(42000));
    operator.b().whileTrue(s_Elevator.positionCmd(300));
    operator.povUp().whileTrue(s_Elevator.positionCmd(28000));
    operator.povLeft().whileTrue(s_Arm.posCmd(0.0));
    // Down
    operator.povDown().whileTrue(s_Arm.posCmd(-72500));
    // Slider
    operator.povRight().whileTrue(s_Arm.posCmd(-38000));
    //operator.povUp().whileTrue(s_Arm.posCmd(30000));

    // Ramp
    operator.leftBumper().whileTrue(s_Arm.posCmd(-40000));
    // 3rd
    operator.rightBumper().whileTrue(s_Arm.posCmd(40000));
    s_Arm.setDefaultCommand(
        s_Arm.moveAndorHoldCommand(
            () ->
                Constants.Arm.backSpeed * operator.getLeftTriggerAxis()
                    + Constants.Arm.frontSpeed * operator.getRightTriggerAxis()));
  }

  private void dualshockOperatorConfiguration() {
    /*ERROR DO NOT DEPLOY, WIP
    COMMENT OUT FOLLOWING PARAGRAPH FOR OLD BEHAVIOR*/
    final var operator = new CommandPS4Controller(operatorPort);
    /*operator.square().whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinInCmd()));
    operator.triangle().whileTrue(s_Intake.pistonsConeCmd().andThen(s_Intake.spinInCmd()));
    operator.circle().whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinEjectCmd()));
    operator.cross().whileTrue(s_Elevator.positionCmd(100).alongWith(s_Arm.posCmd(0)));
    operator.povDown().whileTrue(s_Arm.posCmd(-72750));
    operator.povLeft().whileTrue(s_Arm.)


    operator.share().onTrue(s_Intake.pistonsConeCmd());
    operator.options().onTrue(s_Intake.pistonsCubeCmd());
    operator.cross().whileTrue(s_Intake.spinInCmd());
    operator.square().whileTrue(s_Intake.spinEjectCmd());
*/
    // completely out: 42000
    operator.triangle().whileTrue(s_Elevator.positionCmd(42000));
    operator.circle().whileTrue(s_Elevator.positionCmd(300));
    operator.povUp().whileTrue(s_Elevator.positionCmd(28000));
    operator.povLeft().whileTrue(s_Arm.posCmd(0.0));
    // Down
    operator.povDown().whileTrue(s_Arm.posCmd(-72500));
    // Slider
    operator.povRight().whileTrue(s_Arm.posCmd(-38000));
    //operator.povUp().whileTrue(s_Arm.posCmd(30000));

    // Ramp
    operator.L1().whileTrue(s_Arm.posCmd(-40000));
    // 3rd
    operator.R1().whileTrue(s_Arm.posCmd(40000));
    s_Arm.setDefaultCommand(
        s_Arm.moveAndorHoldCommand(
            () ->
                Constants.Arm.backSpeed * operator.getL2Axis()
                    + Constants.Arm.frontSpeed * operator.getR2Axis()));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
      
    //exampleAuto(s_Swerve);
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
