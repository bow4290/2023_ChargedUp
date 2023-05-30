package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.swerve.*;
import frc.robot.subsystems.*;

public class Controls {
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
}
