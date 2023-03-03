package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.*;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.BalanceThing;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
import java.io.File;
import java.nio.file.Files;
import java.util.HashMap;

public class RobotContainer {
  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Elbow s_Elbow = new Elbow();
  private final Elevator s_Elevator = new Elevator();
  private SwerveAutoBuilder autoBuilder;

  SendableChooser<Command> chooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    configureButtons();
    createAutoBuilder();
    putInfoInDashboard();

    // Create a server for PathPlanner so that the robot pathing can be viewed.
    PathPlannerServer.startServer(5811);

    SmartDashboard.putData(
        "Reset Swerve Modules To Absolute", s_Swerve.resetModulesToAbsoluteCommand());
    SmartDashboard.putData(
        "Reset Elbow and Elevator to Zero",
        Commands.runOnce(
            () -> {
              s_Elbow.resetToZero();
              s_Elevator.resetToZero();
            }));

    chooser.addOption("auto second cube node from lodaing", createAuto("simplehumanpath"));
    chooser.addOption("auto fifth cube node from loading", createAuto("simplefarhumanpath"));
    chooser.addOption("auto second cube node from loading + BALANCE", createAuto("simplehumanpath plus balance"));
    chooser.addOption("auto fifth cube node from loading + BALANCE", createAuto("simplefarhumanpath plus balance"));
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
  }

  /**
   * build.gradle automatically records some information (date, git user, git commit, changed files)
   * whenever the code is built.
   */
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
  // TODO: Detect if a gamepad is PS4/Logitech based on button count?
  private final int driverPort = 0;
  private final boolean driverPS4 = true;
  private final int operatorPort = 1;
  private final boolean operatorPS4 = true;

  private final GenericGamepad driver = GenericGamepad.from(driverPort, driverPS4);
  private final GenericGamepad operator = GenericGamepad.from(operatorPort, operatorPS4);

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

    driver.triangle_y.onTrue(new InstantCommand(s_Swerve::zeroGyro));
    driver.circle_b.whileTrue(new BalanceThing(s_Swerve));

    // Temporarily disabled while it still needs to be fixed-ish
    // driver.cross_a.whileTrue(new GoToNearestScoringLocation(s_Swerve));

    driver.square_x.whileTrue(new AutoBalance(s_Swerve));
  }

  private void operatorConfiguration() {
    operator.square_x.whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinInCmd()));
    operator.triangle_y.whileTrue(s_Intake.pistonsConeCmd().andThen(s_Intake.spinInCmd()));
    operator.circle_b.whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinEjectCmd()));
    operator.cross_a.whileTrue(s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd()));
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
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  private void createAutoBuilder() {
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put(
        "topCone",
        new SequentialCommandGroup(
            s_Elbow.posDegCmd(45),
            s_Elevator.positionMaxCmd(),
            s_Intake.pistonsCubeCmd(),
            s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd()),
            s_Intake.pistonsConeCmd()));

    eventMap.put(
        "topCube",
        new SequentialCommandGroup(
            s_Elbow.posDegCmd(45),
            s_Elevator.positionMaxCmd(),
            s_Intake.spinEjectCmd().withTimeout(0.6),
            s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd())));

    eventMap.put(
      "balance",
      new AutoBalance(s_Swerve)
    );

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    autoBuilder =
        new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPYController, 0.0, 0.0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
            s_Swerve::setModuleStates,
            eventMap,
            true, // Automatically mirror path based on alliance
            s_Swerve);
  }

  private Command createAuto(String name) {
    var pathGroup = PathPlanner.loadPathGroup(name, new PathConstraints(2, 1));

    return 
      new InstantCommand(s_Swerve::gyro180).andThen(autoBuilder.fullAuto(pathGroup));
  }
}
