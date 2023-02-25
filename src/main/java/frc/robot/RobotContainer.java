package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.swerve.BalanceThing;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import java.io.File;
import java.nio.file.Files;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonPoseEstimator;

public class RobotContainer {

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Elbow s_Elbow = new Elbow();
  private final Elevator s_Elevator = new Elevator();
  private final Vision s_Vision = new Vision();
  public static PhotonPoseEstimator photonPoseEstimator;

  SendableChooser<Command> chooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    photonPoseEstimator = s_Vision.photonPoseEstimator; // TODO: find a better way to integrate this
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
    chooser.addOption("do nothing", new InstantCommand(() -> {}));

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

    // It is not intended for the driver to manually operate the elevator during normal robot
    // operation.
    // This should only be used in the event of an unexpected situation.
    DoubleSupplier combined =
        () ->
            operator.leftTrigger.getAsDouble() * Constants.Elevator.retractSpeed
                + operator.rightTrigger.getAsDouble() * Constants.Elevator.extendSpeed;

    new Trigger(() -> Math.abs(combined.getAsDouble()) > Constants.Elevator.elevatorDeadband)
        .whileTrue(s_Elevator.moveCmd(combined));
  }

  private void operatorConfiguration() {
    operator.leftMiddle.onTrue(s_Intake.pistonsConeCmd());
    operator.rightMiddle.onTrue(s_Intake.pistonsCubeCmd());

    operator.a.whileTrue(s_Intake.spinInCmd());
    operator.x.whileTrue(s_Intake.spinEjectCmd());
    operator.y.onTrue(s_Elevator.positionMaxCmd());
    operator.b.onTrue(s_Elevator.positionBaseCmd());

    operator.dpadUp.onTrue(s_Elbow.posDegCmd(0));
    operator.dpadLeft.onTrue(s_Elbow.posDegCmd(-45));
    operator.dpadRight.onTrue(s_Elbow.posDegCmd(45));
    operator.dpadDown.onTrue(s_Elevator.positionMidCmd());

    operator.leftBumper.onTrue(s_Elbow.posDegCmd(90));
    operator.rightBumper.onTrue(s_Elbow.posDegCmd(-90));

    DoubleSupplier combined =
        () ->
            operator.leftTrigger.getAsDouble() * Constants.Arm.backSpeed
                + operator.rightTrigger.getAsDouble() * Constants.Arm.frontSpeed;

    new Trigger(() -> Math.abs(combined.getAsDouble()) > Constants.Arm.armDeadband)
        .whileTrue(s_Elbow.moveCmd(combined));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
