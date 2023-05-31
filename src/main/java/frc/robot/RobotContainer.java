package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.*;
import java.io.File;
import java.nio.file.Files;
import java.util.HashMap;

public class RobotContainer {
  public final Swerve s_Swerve = new Swerve();
  public final Intake s_Intake = new Intake();
  public final Elevator s_Elevator = new Elevator();
  public final Elbow s_Elbow = new Elbow(s_Elevator::getPositionPercent);
  public final LED s_LED = new LED();
  private SwerveAutoBuilder autoBuilder;
  private AutoCommands autoCommands = new AutoCommands(s_Swerve, s_Intake, s_Elbow, s_Elevator);

  SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true); // Remove annoying warnings

    configureButtons();
    createAutoBuilder();
    populateAutoChooser();
    putInfoInDashboard();
    putOtherThingsInDashboard();
    // Create a server for PathPlanner so that the robot pathing can be viewed.
    PathPlannerServer.startServer(5811);

    // s_Intake.intakeHasThing.whileTrue(s_LED.setLEDsCommand(Color.kPurple));
    // s_Intake.intakeHasThing.negate().whileTrue(s_LED.setLEDsCommand(Color.kRed));
    // DO NOT UNCOMMENT THE FOLLOWING LINES
    // robot.explode();
  }

  /* Controllers */
  // TODO: Detect if a gamepad is PS4/Logitech based on button count?
  private final int driverPort = 0;
  private final boolean driverPS4 = true; // testing stuff
  private final int operatorPort = 1;
  private final boolean operatorPS4 = true;
  private final int keyboardApplePort = 2;

  public final GenericGamepad driver = GenericGamepad.from(driverPort, driverPS4);
  public final GenericGamepad operator = GenericGamepad.from(operatorPort, operatorPS4);
  private final CommandGenericHID keyboard = new CommandGenericHID(keyboardApplePort);

  // Java stuff, creates a new Controls object
  Controls controls = new Controls();

  private void configureButtons() {
    // We can actually call both of these since they are on different ports. If concurrent commands
    // run they will interrupt.

    // we call from controls, which gets the code from our Controls.java class
    controls.driverConfiguration(this);
    // operatorConfigurationAppleKeyboard();
    controls.operatorConfiguration(this);
  }

  private void operatorConfigurationAppleKeyboard() {
    // 1 -> third
    // 2 -> forward (reverse) second
    // 3 -> reverse third (cube)
    // 4 -> second
    // 15, 16, 17, 18: cube version of above.
    // 5 -> battery single intake
    // 6 -> battery double intake
    // 7 -> single intake
    // 8 -> double intake
    // 9 & 14 -> ground
    // 11 -> spin intake in
    // 12 -> spin intake out
    // 13 -> reset arm
    // 19 -> elevator down
    // 20 -> elevator up
    // 21 -> arm down
    // 22 -> arm up
    // 23 -> elevator max
    // 24 -> elevator middle
    // 25 -> elevator base
    // 26 -> elbow third
    // 27 -> elbow vertical
    // 28 -> elbow second
    keyboard.button(1).whileTrue(new RobotState().elbowThird().elevatorMax().build(this));
    keyboard.button(2).whileTrue(new RobotState().elbowThird().elevatorMiddle().build(this));
    keyboard.button(3).whileTrue(new RobotState().elbowSecond().elevatorMax().build(this));
    keyboard.button(4).whileTrue(new RobotState().elbowSecond().elevator(0.25).build(this));

    keyboard
        .button(15)
        .whileTrue(
            new RobotState()
                .elbowThirdWeak()
                .elevator(0.7)
                .build(this)
                .andThen(new RobotState().intakeEject().build(this)));
    keyboard
        .button(16)
        .whileTrue(
            new RobotState()
                .elbowThirdWeak()
                .elevatorBase()
                .build(this)
                .andThen(new RobotState().intakeEject().build(this)));
    keyboard
        .button(17)
        .whileTrue(
            new RobotState()
                .elbowSecondWeak()
                .elevator(0.7)
                .build(this)
                .andThen(new RobotState().intakeEject().build(this)));
    keyboard
        .button(18)
        .whileTrue(
            new RobotState()
                .elbowSecondWeak()
                .elevatorBase()
                .build(this)
                .andThen(new RobotState().intakeEject().build(this)));

    keyboard
        .button(5)
        .whileTrue(new RobotState().elbowSingleBattery().elevatorBase().intakeIn().build(this));
    keyboard
        .button(6)
        .whileTrue(new RobotState().elbowDoubleBattery().elevatorMax().intakeIn().build(this));
    keyboard
        .button(7)
        .whileTrue(new RobotState().elbowSingleForward().elevatorBase().intakeIn().build(this));
    keyboard
        .button(8)
        .whileTrue(new RobotState().elbowDoubleForward().elevatorMax().intakeIn().build(this));

    keyboard.button(9).whileTrue(new RobotState().elbowGround().elevatorBase().build(this));
    keyboard.button(14).whileTrue(new RobotState().elbowGround().elevatorBase().build(this));
    keyboard.button(11).whileTrue(new RobotState().intakeIn().build(this));
    keyboard.button(12).whileTrue(new RobotState().intakeEject().build(this));
    keyboard.button(13).whileTrue(new RobotState().elbowBase().elevatorBase().build(this));

    keyboard.button(19).whileTrue(s_Elevator.moveCmd(() -> -0.15));
    keyboard.button(20).whileTrue(s_Elevator.moveCmd(() -> 0.15));
    keyboard.button(21).whileTrue(s_Elbow.moveCmd(() -> -0.15));
    keyboard.button(22).whileTrue(s_Elbow.moveCmd(() -> 0.15));

    keyboard.button(23).whileTrue(new RobotState().elevatorMax().build(this));
    keyboard.button(24).whileTrue(new RobotState().elevatorMiddle().build(this));
    keyboard.button(25).whileTrue(new RobotState().elevatorBase().build(this));
    keyboard.button(26).whileTrue(new RobotState().elbowThird().build(this));
    keyboard.button(27).whileTrue(new RobotState().elbowBase().build(this));
    keyboard.button(28).whileTrue(new RobotState().elbowSecond().build(this));
  }


  // elbow in degrees, 0 is up.
  // elevator [0, 1]
  // intake [-1, 1] positive for in, negative for out
  // sustain true to maintain state, false to end command once achieved
  // min time for spin intake at least x seconds even when state achieved early (for sustain =
  // false)
  // pass null for keep/don't care
  // I miss monads :(
  public Command achieveState(
      Double elbow,
      Double elevator,
      Boolean cone,
      Double intake,
      Boolean thenBrake,
      boolean sustain,
      double minTime) {
    if (elevator != null) elevator = s_Elevator.percentToPosition(elevator);
    Command cmd = (elbow != null) ? s_Elbow.goToDeg(elbow) : new InstantCommand();
    Command cmd2 = (elbow != null) ? s_Elbow.goToDeg(elbow) : new InstantCommand();
    cmd =
        (elevator != null)
            ?
            // ? false // (elevator < 1.0)
            //   ? s_Elevator.smartPos(cmd, cmd2.repeatedly(), elevator)
            // :
            cmd.alongWith(s_Elevator.positionCmd(elevator))
            : cmd;

    Command sus = (elbow != null) ? s_Elbow.goToDegUnending(elbow) : new InstantCommand();
    sus = (elevator != null) ? sus.alongWith(s_Elevator.positionCmd(elevator).repeatedly()) : sus;
    cmd =
        (sustain)
            ? cmd.andThen(sus.repeatedly())
            : (minTime > 0.0) ? cmd.andThen(sus.repeatedly().withTimeout(minTime)) : cmd;
    cmd =
        (cone != null)
            ? (cone)
                ? cmd.beforeStarting(s_Intake.pistonsConeCmd())
                : cmd.beforeStarting(s_Intake.pistonsCubeCmd())
            : cmd;
    cmd =
        (intake != null)
            ? cmd.deadlineWith(
                s_Intake.runEnd(
                    () -> s_Intake.spin(intake),
                    (thenBrake != null && thenBrake)
                        ? s_Intake::retainPositionCmd
                        : s_Intake::stopSpinning))
            : cmd;
    return cmd;
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

    eventMap.put("eject", s_Intake.autoEjectCmd());

    eventMap.put("balance", new AutoBalance(s_Swerve, new Rotation2d(0, -1)));
    eventMap.put("balanceminus", new AutoBalance(s_Swerve, new Rotation2d(0, 1)));
    eventMap.put("intakeCube", autoCommands.intakeCube());
    eventMap.put("intakeFlippedCone", autoCommands.intakeFlippedCone());
    eventMap.put("intakeUp", s_Elbow.goToDeg(0).alongWith(s_Elevator.goToBase()));

    eventMap.put("prepRam", autoCommands.prepRam());
    eventMap.put("ram", autoCommands.ram());

    /*var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);*/

    autoBuilder =
        new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(4.0, 0.0, 0.3),
            new PIDConstants(4.0, 0.0, 0.3),
            s_Swerve::setModuleStates,
            eventMap,
            true, // Automatically mirror path based on alliance
            s_Swerve);
  }

  private void populateAutoChooser() {
    var deployDir = Filesystem.getDeployDirectory();

    try {
      System.out.println("PATH " + deployDir.toPath().resolve("pathplanner").toString());
      // Automatically list all the paths and add them all!
      Files.list(deployDir.toPath().resolve("pathplanner"))
          .sorted()
          .filter(file -> !file.toString().contains("unused"))
          .forEach(
              file -> {
                try {
                  var name = file.getName(file.getNameCount() - 1).toString().replace(".path", "");
                  chooser.addOption(name, createAuto(name));
                } catch (Exception e) {
                  SmartDashboard.putString("ERROR LOADING " + file, e.getMessage());
                }
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
  }

  private Command createAuto(String name) {
    var pathGroup = PathPlanner.loadPathGroup(name, new PathConstraints(4, 4));
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

  private void putOtherThingsInDashboard() {
    SmartDashboard.putData(
        "Reset Swerve Modules To Absolute",
        s_Swerve.resetModulesToAbsoluteCommand().ignoringDisable(true));
    SmartDashboard.putData(
        "Reset Elbow and Elevator to Zero Better",
        Commands.runOnce(
                () -> {
                  s_Elbow.resetToZero();
                  s_Elevator.resetToZero();
                })
            .ignoringDisable(true));

    SmartDashboard.putData("Limelight Drive Mode", s_Swerve.vision.setLLDriverCmd());

    SmartDashboard.putData("CHOOSE AUTO", chooser);
    SmartDashboard.putData(
        "RESEND CHOOSER", new InstantCommand(() -> SmartDashboard.putData("CHOOSE AUTO", chooser)));
  }
}
