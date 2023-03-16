package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

public class Balance extends PIDCommand {
  private Swerve swerve;

  public Balance(Swerve swerve) {
    super(
        new PIDController(2.3, 0.02, 7),
        // Based on pitch
        swerve::getTiltMagnitude,
        // We want to gain a pitch of 0
        0,
        // Output moves bot
        output -> {
          output = Math.abs(output);
          Rotation2d rot = swerve.getTiltDirection();
          // if (output > 0.04) {
          swerve.drive(
              new Translation2d(output * rot.getCos(), output * rot.getSin()), 0, false, true);
          // }
        },
        swerve);
    this.swerve = swerve;

    // Set the controller tolerance - the velocity tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(0.05, 2);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (!interrupted) swerve.lockModules();
  }
}
