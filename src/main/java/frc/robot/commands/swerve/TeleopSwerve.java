package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier forwardBackwardSup;
  private DoubleSupplier leftRightSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier robotSpecificRotationSup;
  private DoubleSupplier specificRotationSup;

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier forwardBackwardSup,
      DoubleSupplier leftRightSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier robotSpecificRotationSup,
      DoubleSupplier specificRotationSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.forwardBackwardSup = forwardBackwardSup;
    this.leftRightSup = leftRightSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.robotSpecificRotationSup = robotSpecificRotationSup;
    this.specificRotationSup = specificRotationSup;
  }

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier forwardBackwardSup,
      DoubleSupplier leftRightSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.forwardBackwardSup = forwardBackwardSup;
    this.leftRightSup = leftRightSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.robotSpecificRotationSup = () -> false;
    this.specificRotationSup = () -> 0;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double forwardBackwardVal =
        MathUtil.applyDeadband(forwardBackwardSup.getAsDouble(), Constants.stickDeadband);
    double leftRightVal =
        MathUtil.applyDeadband(leftRightSup.getAsDouble(), Constants.stickDeadband);

    double rotationVal;
    if (robotSpecificRotationSup.getAsBoolean()) {

      double wantedRotation = specificRotationSup.getAsDouble();
      double currentRotation = s_Swerve.getPose().getRotation().getDegrees() % 360;
      double error = wantedRotation - currentRotation;

      while (error > 180) {
        error -= 360;
      }
      while (error < -180) {
        error += 360;
      }

      error = Math.toRadians(error);

      rotationVal = error * 0.8; // this is technically PID control (without ID control)
      rotationVal = Math.min(Math.max(rotationVal, -1), 1); // prevent tipping the bot?
    } else {
      rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    }

    /* Drive */
    s_Swerve.drive(
        // Intentionally flipped (I think?)
        new Translation2d(forwardBackwardVal, leftRightVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
