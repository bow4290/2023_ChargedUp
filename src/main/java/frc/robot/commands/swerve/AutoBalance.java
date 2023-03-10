package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends SequentialCommandGroup {
  public AutoBalance(Swerve s_Swerve) {
    // Basically: we aim to get on the charge station, and then once we're on it, do BalanceThing to
    // secure ourselves
    // NOTE: This assumes that the robot is pretty close to, but not already on, the far side of the
    // charge station
    super(
        new TeleopSwerve(s_Swerve, () -> -1, () -> 0, () -> 1, () -> false)
            .until(new Trigger(() -> s_Swerve.getTiltMagnitude() > 0.19).debounce(0.3))
            .withTimeout(3),
        new BalanceThing(s_Swerve));
  }
}
