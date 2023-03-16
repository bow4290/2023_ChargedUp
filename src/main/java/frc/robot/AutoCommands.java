package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.math.RateMeasurer;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
  private Swerve s_Swerve;
  private Intake s_Intake;
  private Elbow s_Elbow;
  private Elevator s_Elevator;

  public AutoCommands(Swerve s_Swerve, Intake s_Intake, Elbow s_Elbow, Elevator s_Elevator) {
    this.s_Swerve = s_Swerve;
    this.s_Intake = s_Intake;
    this.s_Elbow = s_Elbow;
    this.s_Elevator = s_Elevator;
  }

  public Command intakeCube() {
    return s_Intake
        .pistonsCubeCmd()
        .andThen(
            s_Elbow
                .goToDeg(-87)
                .alongWith(s_Elevator.goToBase())
                .alongWith(
                    s_Intake.spinInCmd().withTimeout(6).andThen(s_Intake.retainPositionCmd())));
  }

  public Command baseArmAndElevator() {
    return s_Elbow.goToDeg(0).alongWith(s_Elevator.goToBase());
  }

  public Command high() {
    return s_Elevator.goToMax().alongWith(Commands.waitSeconds(0.5).andThen(s_Elbow.goToDeg(45)));
  }

  public Command topCone() {
    return Commands.sequence(
        high(), s_Intake.pistonsCubeCmd(), baseArmAndElevator(), s_Intake.pistonsConeCmd());
  }

  /// Like top cone, but don't put arm back afterwards.
  /// Used in two piece autos because we will bring the arm down while moving.
  public Command topConeAbridged() {
    return Commands.sequence(
        high(), s_Intake.pistonsCubeCmd(), baseArmAndElevator().withTimeout(0.5));
  }

  public Command topCube() {
    return Commands.sequence(high(), s_Intake.autoEjectCmd(), baseArmAndElevator());
  }

  public Command topCubePreparation() {
    return s_Intake.retainPositionCmd().raceWith(baseArmAndElevator().withTimeout(0.5));
  }

  public Command attemptBalance() {
    RateMeasurer measurer = new RateMeasurer();
    return new FunctionalCommand(
        () -> measurer.init(s_Swerve.getTiltMagnitude()),
        () -> {
          double[] grav = s_Swerve.getGravity();
          double magnitude = Math.sqrt(grav[0] * grav[0] + grav[1] * grav[1]);
          Rotation2d rot = new Rotation2d(grav[0], grav[1]);
          measurer.addMeasurement(magnitude);
          SmartDashboard.putNumber("mag rate", measurer.getRate());
          if (magnitude > 0.08 && Math.abs(measurer.getRate()) < 0.1) {
            double mult = 0.4;
            s_Swerve.drive(
                new Translation2d(mult * rot.getCos(), mult * rot.getSin()), 0, false, false);
          } else {
            s_Swerve.lockModules();
          }
        },
        s_Swerve::lockModules,
        () -> false);
  }
}
