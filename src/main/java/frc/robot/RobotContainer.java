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


  /* Controllers */
  private final int driverPort = 0;
  private final boolean driverDualshock = true;
  private final int operatorPort = 1;
  private final boolean operatorDualshock = true;

  private final GenericGamepad driver = driverDualshock ? new GenericGamepad(new CommandPS4Controller(driverPort)) :
          new GenericGamepad(new CommandXboxController(driverPort));
  private final GenericGamepad operator = operatorDualshock ? new GenericGamepad(new CommandPS4Controller(operatorPort)) :
          new GenericGamepad(new CommandXboxController(operatorPort));

  // TODO: It could be possible to detect whether a gamepad is PS4 or Xbox(Logitech) based on the button count
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
  }

  private void operatorConfiguration() {
    operator.leftMiddle.onTrue(s_Intake.pistonsConeCmd());
    operator.rightMiddle.onTrue(s_Intake.pistonsCubeCmd());
    operator.a.whileTrue(s_Intake.spinInCmd());
    operator.x.whileTrue(s_Intake.spinEjectCmd());

    operator.y.whileTrue(s_Elevator.positionCmd(42000));
    operator.b.whileTrue(s_Elevator.positionCmd(300));
    operator.dpadUp.whileTrue(s_Elevator.positionCmd(28000));
    operator.dpadLeft.whileTrue(s_Arm.posCmd(0.0));
    operator.dpadDown.whileTrue(s_Arm.posCmd(-72500));
    operator.dpadRight.whileTrue(s_Arm.posCmd(-38000));

    // Ramp
    operator.leftBumper.whileTrue(s_Arm.posCmd(-40000));
    // 3rd
    operator.rightBumper.whileTrue(s_Arm.posCmd(40000));
    s_Arm.setDefaultCommand(
        s_Arm.moveOrHoldCmd(
            () ->
                Constants.Arm.backSpeed * operator.leftTrigger.getAsDouble()
                    + Constants.Arm.frontSpeed * operator.rightTrigger.getAsDouble()));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
