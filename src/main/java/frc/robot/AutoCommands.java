package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command spinIntake() {
    return s_Intake.spinInCmd().withTimeout(8).andThen(s_Intake.retainPositionCmd());
  }

  public Command intakeCube() {
    return s_Intake
        .pistonsCubeCmd()
        .andThen(s_Elbow.groundPosition().alongWith(s_Elevator.goToBase(), spinIntake()));
  }

  public Command intakeFlippedCone() {
    return s_Intake
        .pistonsConeCmd()
        // ver 99
        .andThen(s_Elbow.goToDeg(-98).alongWith(s_Elevator.goToBase(), spinIntake()));
  }

  public Command baseArmAndElevator() {
    return s_Elbow
        .goToDeg(0)
        .alongWith(s_Elevator.goToBase().beforeStarting(Commands.waitSeconds(0.4)));
  }

  public Command quickReset() {
    return baseArmAndElevator().withTimeout(0.2);
  }

  public Command high() {
    return s_Elevator
        .goToMax()
        .alongWith(
            s_Elbow
                .goToDeg(Constants.Elbow.Positions.third)
                .beforeStarting(Commands.waitSeconds(0.3)));
  }

  public Command topCone() {
    return Commands.sequence(
        high(), s_Intake.autoEjectCmd(), baseArmAndElevator(), s_Intake.pistonsConeCmd());
  }

  /// Like top cone, but don't put arm back afterwards.
  /// Used in two piece autos because we will bring the arm down while moving.
  public Command topConeAbridged() {
    return Commands.sequence(
        high(), s_Intake.pistonsCubeCmd(), s_Intake.autoEjectCmd(), quickReset());
  }

  public Command topCube() {
    return Commands.sequence(high(), s_Intake.autoEjectCmd(), baseArmAndElevator());
  }

  public Command topCubePreparation() {
    return high();
  }

  public Command topCubeSecond() {
    return high().andThen(s_Intake.autoEjectCmd(), quickReset());
  }

  public Command prepRam() {
    return s_Elbow.goToDeg(0);
  }

  public Command ram() {
    return s_Elbow
        .goToDeg(10)
        .alongWith(
            s_Intake
                .pistonsCubeCmd()
                .andThen(s_Intake.runEnd(() -> s_Intake.spin(-1.0), s_Intake::stopSpinning)));
  }
}
