package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

public class BalanceThing extends PIDCommand {
  public BalanceThing(Swerve swerve) {
    super(
        new PIDController(2.3, 0.01, 10),
        // Based on pitch
        swerve::getTiltMagnitude,
        // We want to gain a pitch of 0
        0,
        // Output moves bot
        output -> {
          output = Math.abs(output);
          if (output > 0.05) {
            Rotation2d rot = swerve.getTiltDirection();
            swerve.drive(
                new Translation2d(output * rot.getCos(), output * rot.getSin()), 0, false, true);
          } else {
            swerve.lockChargeStation();
          }
        },
        swerve);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(0.05, 2);
    // Account for wpilib bug?
    getController().setSetpoint(0);
  }
  /*
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean insuccessful) {
    super.end(insuccessful);
    if (!insuccessful)
      swerve.lockChargeStation();
  }
  */
}
