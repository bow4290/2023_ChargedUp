package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends SequentialCommandGroup {
  public static PIDController makePID() {
    final var PIDC = new PIDController(5, 0.01, 500);
    // 0.04 ~= sin(2.5deg)
    PIDC.setTolerance(0.04, 0.05);
    return PIDC;
  }

  static PIDController PIDC = makePID();

  public AutoBalance(Swerve s_Swerve, Rotation2d dir) {
    // Basically: we aim to get on the charge station, and then once we're on it, do PID balance to
    // secure ourselves. This assumes that the robot is pretty close to, but not already on, the
    // side of the charge station
    super(
        /*new TeleopSwerve(s_Swerve, dir::getSin, dir::getCos, () -> 0.7, () -> false)
        .until(new Trigger(() -> s_Swerve.getTiltMagnitude() > 0.2).debounce(0.3))
        .withTimeout(0.6),*/
        // Commands.run(PIDC::reset),
        new PIDCommand(
                PIDC,
                // Based on pitch
                s_Swerve::getTiltMagnitude,
                // We want to gain a pitch of 0
                0,
                // Output moves bot
                output -> {
                  output = Math.abs(output);
                  Rotation2d rot = s_Swerve.getTiltDirection();
                  if (output > 0.05) {
                    s_Swerve.drive(
                        new Translation2d(output * rot.getCos(), output * rot.getSin()),
                        0,
                        false,
                        true);
                  } else {
                    // This technically sets the motors to brake mode?
                    s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
                  }
                },
                s_Swerve)
            // .until(PIDC::atSetpoint)
            .andThen(s_Swerve.lockModulesCommand()));
  }
}
