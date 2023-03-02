package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.*;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.BalanceThing;
import frc.robot.commands.swerve.GoToNearestScoringLocation;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import java.io.File;
import java.nio.file.Files;
import org.photonvision.PhotonPoseEstimator;

public class RobotContainer {

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Elbow s_Elbow = new Elbow();
  private final Elevator s_Elevator = new Elevator();
  public static PhotonPoseEstimator photonPoseEstimator;

  SendableChooser<Command> chooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    configureButtons();
    putInfoInDashboard();

    SmartDashboard.putData(
        "Reset Swerve Modules To Absolute", s_Swerve.resetModulesToAbsoluteCommand());
    SmartDashboard.putData(
        "Reset Elbow/Elevator to Zero",
        new InstantCommand(
            () -> {
              s_Elbow.resetToZero();
              s_Elevator.resetToZero();
            }));

    chooser.setDefaultOption("move forward", new exampleAuto(s_Swerve, s_Elbow));
    chooser.addOption(
        "top auto close to human",
        new topAutoLeft("simplehumanpath", s_Swerve, s_Elbow, s_Elevator, s_Intake));
    chooser.addOption(
        "top auto far from human",
        new topAutoLeft("simplefarhumanpath", s_Swerve, s_Elbow, s_Elevator, s_Intake));

    chooser.addOption(
        "stationary 3rd cone",
        new SequentialCommandGroup(
            s_Elbow.posDegCmd(45),
            s_Elevator.positionMaxCmd(),
            s_Intake.pistonsCubeCmd(),
            s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd()),
            s_Intake.pistonsConeCmd()));
    chooser.addOption("do nothing", new InstantCommand(() -> {}));
    SmartDashboard.putData("choose auto", chooser);
    // DO NOT UNCOMMENT THE FOLLOWING LINES
    // robot.explode();
    // Robot-on-fire incident counter so far: 1
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

  /* Controllers */
  // TODO: It should be possible to detect whether a gamepad is PS4 or Xbox(Logitech) based on the
  // button count
  private final int driverPort = 0;
  private final boolean driverDualshock = true;
  private final int operatorPort = 1;
  private final boolean operatorDualshock = true;

  private final GenericGamepad driver =
      driverDualshock
          ? new GenericGamepad(new CommandPS4Controller(driverPort))
          : new GenericGamepad(new CommandXboxController(driverPort));
  private final GenericGamepad operator =
      operatorDualshock
          ? new GenericGamepad(new CommandPS4Controller(operatorPort))
          : new GenericGamepad(new CommandXboxController(operatorPort));

  private void configureButtons() {
    driverConfiguration();
    operatorConfiguration();
  }

  private void driverConfiguration() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.leftY.getAsDouble(),
            () -> -driver.leftX.getAsDouble(),
            () -> -driver.rightX.getAsDouble(),
            driver.leftBumper::getAsBoolean));

    driver.y.onTrue(new InstantCommand(s_Swerve::zeroGyro));
    driver.b.whileTrue(new BalanceThing(s_Swerve));

    /*driver.a.whileTrue(
    new GoToPoint(
        s_Swerve,
        s_Swerve.getPose(), // WOW I JUST FIGURED OUT WHY THIS ISN'T WORKING
        new Pose2d(new Translation2d(1.9, 2.75), new Rotation2d(0))));*/
    driver.a.whileTrue(new GoToNearestScoringLocation(s_Swerve));

    driver.x.whileTrue(new AutoBalance(s_Swerve));
    // s
    // It is not intended for the driver to manually operate the elevator during normal robot
    // operation.
    // This should only be used in the event of an unexpected situation.
    /*DoubleSupplier combined =
          () ->
              driver.leftTrigger.getAsDouble() * Constants.Elevator.retractSpeed
                  + driver.rightTrigger.getAsDouble() * Constants.Elevator.extendSpeed;

      new Trigger(() -> Math.abs(combined.getAsDouble()) > Constants.Elevator.elevatorDeadband)
          .whileTrue(s_Elevator.moveCmd(combined));
    */
  }

  private void operatorConfiguration() {
    operator.x.whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinInCmd()));
    operator.y.whileTrue(s_Intake.pistonsConeCmd().andThen(s_Intake.spinInCmd()));
    operator.b.whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinEjectCmd()));
    operator.a.whileTrue(s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd()));
    operator.dpadDown.whileTrue(s_Elbow.posDegCmd(-85).alongWith(s_Elevator.positionBaseCmd()));
    operator.dpadLeft.whileTrue(s_Elevator.positionBaseCmd());
    operator.dpadUp.whileTrue(s_Elevator.positionMidCmd());
    operator.dpadRight.whileTrue(s_Elevator.positionMaxCmd());
    // 2nd row
    operator.leftBumper.whileTrue(s_Elbow.posDegCmd(-38));
    // reverse
    operator.leftTriggerB.whileTrue(s_Elbow.posDegCmd(39.5));
    // operator
    operator.rightBumper.whileTrue(s_Elbow.posDegCmd(-45));
    // ramp
    operator.rightTriggerB.whileTrue(s_Elbow.posDegCmd(-46.5));
    operator.rightJoystickPushed.whileTrue(s_Elbow.posDegCmd(48.5));
    operator.leftMiddle.onTrue(s_Intake.pistonsConeCmd());
    operator.rightMiddle.onTrue(s_Intake.pistonsCubeCmd());

    // operator.a.whileTrue(s_Intake.spinInCmd());
    // operator.x.whileTrue(s_Intake.spinEjectCmd());
    // operator.y.onTrue(s_Elevator.positionMaxCmd());
    // operator.b.onTrue(s_Elevator.positionBaseCmd());

    // operator.dpadUp.onTrue(s_Elbow.posDegCmd(0));
    // operator.dpadLeft.onTrue(s_Elbow.posDegCmd(-45));
    // operator.dpadRight.onTrue(s_Elbow.posDegCmd(45));

    // operator.dpadDown.onTrue(s_Elevator.positionMidCmd());
    // // -43141
    // // operator.leftBumper.onTrue(s_Elbow.posDegCmd(90)); DO NOT USE
    // // -57.5821
    // operator.rightBumper.onTrue(s_Elbow.posDegCmd(-85));
    // // operator.leftBumper.whileTrue(s_Elevator.moveCmd(Constants.Elevator.retractSpeed));
    // // operator.rightBumper.whileTrue(s_Elevator.moveCmd(Constants.Elevator.extendSpeed));

    // DoubleSupplier combined =
    //     () ->
    //         operator.leftTrigger.getAsDouble() * Constants.Arm.backSpeed
    //             + operator.rightTrigger.getAsDouble() * Constants.Arm.frontSpeed;

    // new Trigger(() -> Math.abs(combined.getAsDouble()) > Constants.Arm.armDeadband)
    //     .whileTrue(s_Elbow.moveCmd(combined));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
