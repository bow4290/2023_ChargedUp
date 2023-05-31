package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.*;

public class Controls {
  // Determines whether or not to use Apple's keyboard operator config or not
  public final boolean useAppleConfig = false;

  public void driverConfiguration(RobotContainer bot) {
    bot.s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            bot.s_Swerve,
            () -> {
              double input = -bot.driver.leftY.getAsDouble();
              return Math.copySign(Math.pow(input, Constants.driveSens), input);
            },
            () -> {
              double input = -bot.driver.leftX.getAsDouble();
              return Math.copySign(Math.pow(input, Constants.driveSens), input);
            },
            () -> {
              double input = -bot.driver.rightX.getAsDouble();
              return Math.copySign(Math.pow(input, Constants.turnSens), input);
            },
            bot.driver.leftBumper,
            bot.driver.rightBumper.and(
                () ->
                    Math.hypot(bot.driver.rightX.getAsDouble(), bot.driver.rightY.getAsDouble())
                        > 0.2),
            () ->
                Math.round(
                        (Math.toDegrees(
                                    Math.atan2(
                                        -bot.driver.rightY.getAsDouble(),
                                        bot.driver.rightX.getAsDouble()))
                                - 90)
                            / 90)
                    * 90));

    bot.driver.triangle_y.onTrue(new InstantCommand(bot.s_Swerve::zeroGyro));
    // bot.driver.circle_b.whileTrue(new BalanceThing(bot.s_Swerve));

    // Temporarily disabled while it still needs to be fixed-ish
    // bot.driver.cross_a.whileTrue(new GoToNearestScoringLocation(bot.s_Swerve));
    // bot.driver.cross_a.whileTrue(autoCommands.attemptBalance());
    bot.driver.circle_b.whileTrue(bot.s_Swerve.lockModulesCommand());
    // bot.driver.cross_a.whileTrue(autoCommands.intakeCube());
    // bot.driver.rightBumper.whileTrue(autoCommands.topCube());
    bot.driver.cross_a.onTrue(bot.s_LED.setLEDsCommand(Color.kBlue));
    bot.driver.square_x.onTrue(bot.s_LED.setLEDsCommand(Color.kRed));
    // bot.driver.square_x.whileTrue(new AutoBalance(bot.s_Swerve, new Rotation2d(0, 1)));
  }

  public void operatorConfiguration(RobotContainer bot) {

    // Pistons to cube, intake spin in
    bot.operator.square_x.whileTrue(
        bot.s_Intake.pistonsCubeCmd().andThen(bot.s_Intake.spinInCmd()));
    // Pistons to cone, intake spin in
    bot.operator.triangle_y.whileTrue(
        bot.s_Intake.pistonsConeCmd().andThen(bot.s_Intake.spinInCmd()));
    // Pistons to cube, intake spin out (eject)
    bot.operator.circle_b.whileTrue(
        bot.s_Intake.pistonsCubeCmd().andThen(bot.s_Intake.spinEjectCmd()));
    // Arm to vertical, elevator to base
    bot.operator.cross_a.whileTrue(
        bot.s_Elevator.smartBase(bot.s_Elbow.goToDeg(0), bot.s_Elbow.goToDegUnending(0)));
    // Intake position from battery side
    bot.operator.dpadDown.whileTrue(
        bot.s_Elevator.smartBase(
            bot.s_Elbow.groundPosition(), bot.s_Elbow.groundPosition().repeatedly()));
    // Elevator to base
    bot.operator.dpadLeft.whileTrue(bot.s_Elevator.goToBase());
    // Elevator to mid
    bot.operator.dpadUp.whileTrue(bot.s_Elevator.goToMid());
    // Elevator to max
    bot.operator.dpadRight.whileTrue(bot.s_Elevator.goToMax());
    // Arm back battery side for 2nd row
    bot.operator.leftBumper.whileTrue(
        bot.s_Elbow.secondPosition().alongWith(bot.s_Elevator.goToMid()));
    // Arm out front side for 3rd row
    bot.operator.leftTriggerB.whileTrue(
        bot.s_Elevator
            .goToMax()
            .alongWith(bot.s_Elbow.goToDegUnending(49).beforeStarting(Commands.waitSeconds(0.3))));
    // Arm out battery side, human player double (platform)
    bot.operator.rightBumper.whileTrue(bot.s_Elbow.goToDegUnending(-55));
    // Arm out battery side, human player single (ramp) cube
    bot.operator.rightTriggerB.whileTrue(bot.s_Elbow.goToDegUnending(-59));
    // Arm out front side, human player single (ramp) cone
    bot.operator.rightMiddle.whileTrue(bot.s_Elbow.goToDegUnending(57));
    // Intake but slightly lower
    bot.operator.leftMiddle.whileTrue(
        bot.s_Elbow.goToDegUnending(-105).alongWith(bot.s_Elevator.goToBase()));
    // Pistons to cone
    /*operator.leftMiddle.onTrue(s_Intake.pistonsConeCmd());
    // Pistons to cube
    operator.rightMiddle.onTrue(s_Intake.pistonsCubeCmd());*/

    bot.operator.leftJoystickPushed.whileTrue(
        bot.s_Elevator.moveCmd(
            () ->
                bot.operator.leftY.getAsDouble()
                    * (bot.operator.topMiddle.getAsBoolean() ? -1 : -0.3)));
    bot.operator.rightJoystickPushed.whileTrue(
        bot.s_Elbow.moveCmd(
            () ->
                bot.operator.rightX.getAsDouble()
                    * (bot.operator.topMiddle.getAsBoolean() ? 1 : 0.2)));
  }

  public void operatorConfigurationAppleKeyboard(RobotContainer bot) {
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
    bot.keyboard.button(1).whileTrue(new RobotState().elbowThird().elevatorMax().build(bot));
    bot.keyboard.button(2).whileTrue(new RobotState().elbowThird().elevatorMiddle().build(bot));
    bot.keyboard.button(3).whileTrue(new RobotState().elbowSecond().elevatorMax().build(bot));
    bot.keyboard.button(4).whileTrue(new RobotState().elbowSecond().elevator(0.25).build(bot));

    bot.keyboard
        .button(15)
        .whileTrue(
            new RobotState()
                .elbowThirdWeak()
                .elevator(0.7)
                .build(bot)
                .andThen(new RobotState().intakeEject().build(bot)));
    bot.keyboard
        .button(16)
        .whileTrue(
            new RobotState()
                .elbowThirdWeak()
                .elevatorBase()
                .build(bot)
                .andThen(new RobotState().intakeEject().build(bot)));
    bot.keyboard
        .button(17)
        .whileTrue(
            new RobotState()
                .elbowSecondWeak()
                .elevator(0.7)
                .build(bot)
                .andThen(new RobotState().intakeEject().build(bot)));
    bot.keyboard
        .button(18)
        .whileTrue(
            new RobotState()
                .elbowSecondWeak()
                .elevatorBase()
                .build(bot)
                .andThen(new RobotState().intakeEject().build(bot)));

    bot.keyboard
        .button(5)
        .whileTrue(new RobotState().elbowSingleBattery().elevatorBase().intakeIn().build(bot));
    bot.keyboard
        .button(6)
        .whileTrue(new RobotState().elbowDoubleBattery().elevatorMax().intakeIn().build(bot));
    bot.keyboard
        .button(7)
        .whileTrue(new RobotState().elbowSingleForward().elevatorBase().intakeIn().build(bot));
    bot.keyboard
        .button(8)
        .whileTrue(new RobotState().elbowDoubleForward().elevatorMax().intakeIn().build(bot));

    bot.keyboard.button(9).whileTrue(new RobotState().elbowGround().elevatorBase().build(bot));
    bot.keyboard.button(14).whileTrue(new RobotState().elbowGround().elevatorBase().build(bot));
    bot.keyboard.button(11).whileTrue(new RobotState().intakeIn().build(bot));
    bot.keyboard.button(12).whileTrue(new RobotState().intakeEject().build(bot));
    bot.keyboard.button(13).whileTrue(new RobotState().elbowBase().elevatorBase().build(bot));

    bot.keyboard.button(19).whileTrue(bot.s_Elevator.moveCmd(() -> -0.15));
    bot.keyboard.button(20).whileTrue(bot.s_Elevator.moveCmd(() -> 0.15));
    bot.keyboard.button(21).whileTrue(bot.s_Elbow.moveCmd(() -> -0.15));
    bot.keyboard.button(22).whileTrue(bot.s_Elbow.moveCmd(() -> 0.15));

    bot.keyboard.button(23).whileTrue(new RobotState().elevatorMax().build(bot));
    bot.keyboard.button(24).whileTrue(new RobotState().elevatorMiddle().build(bot));
    bot.keyboard.button(25).whileTrue(new RobotState().elevatorBase().build(bot));
    bot.keyboard.button(26).whileTrue(new RobotState().elbowThird().build(bot));
    bot.keyboard.button(27).whileTrue(new RobotState().elbowBase().build(bot));
    bot.keyboard.button(28).whileTrue(new RobotState().elbowSecond().build(bot));
  }
}
