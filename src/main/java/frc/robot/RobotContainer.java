package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.*;
import java.io.File;
import java.nio.file.Files;
import java.util.HashMap;

public class RobotContainer {
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Elbow s_Elbow = new Elbow();
  private final Elevator s_Elevator = new Elevator();
  private SwerveAutoBuilder autoBuilder;
  private AutoCommands autoCommands = new AutoCommands(s_Swerve, s_Intake, s_Elbow, s_Elevator);

  SendableChooser<Command> chooser = new SendableChooser<>();

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

    var deployDir = Filesystem.getDeployDirectory();

    try {
      System.out.println("PATH " + deployDir.toPath().resolve("pathplanner").toString());
      // Automatically list all the paths and add them all!
      Files.list(deployDir.toPath().resolve("pathplanner"))
          .sorted()
          .filter(file -> !file.toString().contains("unused"))
          .forEach(
              file -> {
                var name = file.getName(file.getNameCount() - 1).toString().replace(".path", "");
                chooser.addOption(name, createAuto(name));
              });
    } catch (Exception e) {
      // Add manually, even though this should literally never happen
      // Maybe it will happen, though?
      SmartDashboard.putString("WARNING", "UNABLE TO AUTOMATICALLY DO AUTOS");
      e.printStackTrace();
      chooser.addOption("2nd cone", createAuto("2nd cone"));
      chooser.addOption("5th cone", createAuto("5th cone"));
      chooser.addOption("6th cone", createAuto("6th cone"));
      chooser.addOption("1st cube", createAuto("1st cube"));
      chooser.addOption("3rd cube", createAuto("3rd cube"));

      chooser.addOption("2nd cone + balance", createAuto("2nd cone + balance"));
      chooser.addOption("5th cone + balance", createAuto("5th cone + balance"));
      chooser.addOption("6th cone + balance", createAuto("6th cone + balance"));
      chooser.addOption("1st cube + balance", createAuto("1st cube + balance"));
      chooser.addOption("3rd cube + balance", createAuto("3rd cube + balance"));

      chooser.addOption("5th cone + pickup", createAuto("5th cone + pickup"));
    }
    chooser.addOption("stationary cone", autoCommands.topCone());
    chooser.addOption("stationary cube", autoCommands.topCube());

    chooser.setDefaultOption("do nothing", new InstantCommand(() -> {}));

    SmartDashboard.putData("CHOOSE AUTO", chooser);
    SmartDashboard.putData(
        "RESEND CHOOSER", new InstantCommand(() -> SmartDashboard.putData("CHOOSE AUTO", chooser)));

    // DO NOT UNCOMMENT THE FOLLOWING LINES
    // robot.explode();
  }
  /* Controllers */
  // TODO: Detect if a gamepad is PS4/Logitech based on button count?
  private final int driverPort = 0;
  private final boolean driverPS4 = true;
  private final int operatorPort = 1;
  private final boolean operatorPS4 = true;
  private final boolean operatorKabir = true;

  private final GenericGamepad driver = GenericGamepad.from(driverPort, driverPS4);
  private final GenericGamepad operator = GenericGamepad.from(operatorPort, operatorPS4);

  private void configureButtons() {
    driverConfiguration();
    if (operatorKabir) operatorConfigurationKabir();
    else operatorConfiguration();
  }

  private void driverConfiguration() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> {
              double input = -driver.leftY.getAsDouble();
              return Math.copySign(Math.pow(input, Constants.driveSens), input);
            },
            () -> {
              double input = -driver.leftX.getAsDouble();
              return Math.copySign(Math.pow(input, Constants.driveSens), input);
            },
            () -> {
              double input = -driver.rightX.getAsDouble();
              return Math.copySign(Math.pow(input, Constants.turnSens), input);
            },
            driver.leftBumper::getAsBoolean));

    driver.triangle_y.onTrue(new InstantCommand(s_Swerve::zeroGyro));
    // driver.circle_b.whileTrue(new BalanceThing(s_Swerve));

    // Temporarily disabled while it still needs to be fixed-ish
    // driver.cross_a.whileTrue(new GoToNearestScoringLocation(s_Swerve));
    driver.cross_a.whileTrue(autoCommands.attemptBalance());
    driver.circle_b.whileTrue(s_Swerve.lockModulesCommand());

    driver.square_x.whileTrue(new AutoBalance(s_Swerve, new Rotation2d(0, 1)));
  }

  private void operatorConfiguration() {
    // Pistons to cube, intake spin in
    operator.square_x.whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinInCmd()));
    // Pistons to cone, intake spin in
    operator.triangle_y.whileTrue(s_Intake.pistonsConeCmd().andThen(s_Intake.spinInCmd()));
    // Pistons to cube, intake spin out (eject)
    operator.circle_b.whileTrue(s_Intake.pistonsCubeCmd().andThen(s_Intake.spinEjectCmd()));
    // Arm to vertical, elevator to base
    operator.cross_a.whileTrue(s_Elbow.goToDegUnending(0).alongWith(s_Elevator.goToBase()));
    // Intake position from battery side
    operator.dpadDown.whileTrue(s_Elbow.goToDegUnending(-89).alongWith(s_Elevator.goToBase()));
    // Elevator to base
    operator.dpadLeft.whileTrue(s_Elevator.goToBase());
    // Elevator to mid
    operator.dpadUp.whileTrue(s_Elevator.goToMid());
    // Elevator to max
    operator.dpadRight.whileTrue(s_Elevator.goToMax());
    // Arm back battery side for 2nd row
    operator.leftBumper.whileTrue(s_Elbow.goToDegUnending(-48).alongWith(s_Elevator.goToMid()));
    // Arm out front side for 3rd row
    operator.leftTriggerB.whileTrue(
        s_Elevator
            .goToMax()
            .alongWith(s_Elbow.goToDegUnending(43).beforeStarting(Commands.waitSeconds(0.3))));
    // Arm out battery side, human player double (platform)
    operator.rightBumper.whileTrue(s_Elbow.goToDegUnending(-48));
    // Arm out battery side, human player single (ramp) cube
    operator.rightTriggerB.whileTrue(s_Elbow.goToDegUnending(-52));
    // Arm out front side, human player single (ramp) cone
    operator.rightJoystickPushed.whileTrue(s_Elbow.goToDegUnending(50));
    // Intake but slightly lower
    operator.leftJoystickPushed.whileTrue(
        s_Elbow.goToDegUnending(-92).alongWith(s_Elevator.goToBase()));
    // Pistons to cone
    operator.leftMiddle.onTrue(s_Intake.pistonsConeCmd());
    // Pistons to cube
    operator.rightMiddle.onTrue(s_Intake.pistonsCubeCmd());
  }

  private void operatorConfigurationKabir() {
    // Pistons to cube, intake spin in
    operator.square_x.onTrue(s_Intake.pistonsCubeCmd());
    operator.square_x.debounce(0.07).whileTrue(s_Intake.spinInCmd());
    // Pistons to cone, intake spin in
    operator.triangle_y.onTrue(s_Intake.pistonsConeCmd());
    operator.triangle_y.debounce(0.07).whileTrue(s_Intake.spinInCmd());
    // Eject
    operator.circle_b.whileTrue(s_Intake.spinEjectCmd());
    // Base
    operator.cross_a.whileTrue(s_Elbow.goToDegUnending(0).alongWith(s_Elevator.goToBase()));

    // Ground intake
    operator.dpadLeft.whileTrue(s_Elbow.goToDegUnending(-89).alongWith(s_Elevator.goToBase()));
    // Ground intake but LOWER!
    operator.rightBumper.whileTrue(s_Elbow.goToDegUnending(-92).alongWith(s_Elevator.goToBase()));

    // Elevator MAX
    operator.dpadUp.whileTrue(s_Elevator.goToMax());
    // Elevator MID
    operator.dpadRight.whileTrue(s_Elevator.goToMid());
    // Elevator BASE
    operator.dpadDown.whileTrue(s_Elevator.goToBase());

    // Arm and elevator manual control (by pushing)
    operator.leftJoystickPushed.whileTrue(
        s_Elevator.moveCmd(
            () ->
                operator.leftY.getAsDouble() * (operator.bottomMiddle.getAsBoolean() ? -1 : -0.3)));
    operator.rightJoystickPushed.whileTrue(
        s_Elbow.moveCmd(
            () ->
                operator.rightX.getAsDouble() * (operator.bottomMiddle.getAsBoolean() ? 1 : 0.2)));

    // Ramp BACK
    operator.leftMiddle.whileTrue(s_Elbow.goToDegUnending(-52));
    // Ramp FRONT
    operator.rightMiddle.whileTrue(s_Elbow.goToDegUnending(50));

    // Third row FRONT side
    operator.rightTriggerB.whileTrue(
        s_Elevator
            .goToMax()
            .alongWith(s_Elbow.goToDegUnending(43).beforeStarting(Commands.waitSeconds(0.25))));
    // Second row BACK side
    operator.leftTriggerB.whileTrue(
        s_Elevator
            .goToMid()
            .alongWith(s_Elbow.goToDegUnending(-43).beforeStarting(Commands.waitSeconds(0.25))));

    // PLATFORM BACK (Also, don't forget to extend elevator)
    operator.leftBumper.whileTrue(s_Elbow.goToDegUnending(-48));
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  private void createAutoBuilder() {
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("topCone", autoCommands.topCone());
    eventMap.put("topConeAbridged", autoCommands.topConeAbridged());
    eventMap.put("topCube", autoCommands.topCube());
    // TODO: Fix these in the paths
    eventMap.put("topCubePreparation", autoCommands.topCubePreparation());
    eventMap.put("topCubeSecond", autoCommands.topCubeSecond());
    eventMap.put("topCubeMid", s_Elevator.goToMax());
    eventMap.put("baseArmAndElevator", autoCommands.baseArmAndElevator());

    eventMap.put("balance", new AutoBalance(s_Swerve, new Rotation2d(0, -1)));
    eventMap.put("balanceminus", new AutoBalance(s_Swerve, new Rotation2d(0, 1)));
    eventMap.put("intakeCube", autoCommands.intakeCube());
    eventMap.put("intakeUp", s_Elbow.goToDeg(0).alongWith(s_Elevator.goToBase()));

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
    var pathGroup = PathPlanner.loadPathGroup(name, new PathConstraints(3, 3));
    return Commands.sequence(
        Commands.print("Starting auto: " + name), // For debugging / looking through post-match logs
        autoBuilder.fullAuto(pathGroup)
        // That did not work
        /*Commands.runOnce(
        s_Swerve
            ::gyroFlip180)*/ ); // This is necessary because of the way the gyro starts in a
    // match
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
}
