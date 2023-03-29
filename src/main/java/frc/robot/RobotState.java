package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

/// Builder pattern 'cuz java doesn't have structs
public class RobotState {
  Optional<Double> elbow;
  Optional<Double> elevator;
  // True -> Closed, False -> Open
  Optional<Boolean> intakePistons;
  Optional<Double> intakeSpeed;
  Optional<Boolean> intakeBrake;

  public RobotState elbow(double v) {
    elbow = Optional.of(v);
    return this;
  }

  public RobotState elbowThird() {
    return elbow(Constants.Elbow.Positions.third);
  }

  public RobotState elbowThirdWeak() {
    return elbow(Constants.Elbow.Positions.thirdWeak);
  }

  public RobotState elbowSecond() {
    return elbow(Constants.Elbow.Positions.second);
  }

  public RobotState elbowSecondWeak() {
    return elbow(Constants.Elbow.Positions.secondWeak);
  }

  public RobotState elbowGround() {
    return elbow(Constants.Elbow.Positions.ground);
  }

  public RobotState elbowDoubleBattery() {
    return elbow(Constants.Elbow.Positions.humanDoubleBattery);
  }

  public RobotState elbowDoubleForward() {
    return elbow(Constants.Elbow.Positions.humanDoubleForward);
  }

  public RobotState elbowSingleBattery() {
    return elbow(Constants.Elbow.Positions.humanSingleBattery);
  }

  public RobotState elbowSingleForward() {
    return elbow(Constants.Elbow.Positions.humanSingleForward);
  }

  public RobotState elbowBase() {
    return elbow(0.0);
  }

  public RobotState elevator(double v) {
    elevator = Optional.of(v);
    return this;
  }

  public RobotState elevatorBase() {
    return elevator(0.0);
  }

  public RobotState elevatorMiddle() {
    return elevator(0.5);
  }

  public RobotState elevatorMax() {
    return elevator(1.0);
  }

  public RobotState pistons(boolean v) {
    intakePistons = Optional.of(v);
    return this;
  }

  public RobotState pistonsCube() {
    return pistons(false);
  }

  public RobotState pistonsCone() {
    return pistons(true);
  }

  public RobotState intake(double v) {
    intakeSpeed = Optional.of(v);
    return this;
  }

  public RobotState intakeIn() {
    return intakeBrake().intake(Constants.Intake.inSpeed);
  }

  public RobotState intakeEject() {
    return intake(Constants.Intake.ejectSpeed);
  }

  public RobotState intakeBrake() {
    intakeBrake = Optional.of(true);
    return this;
  }

  public Command build(RobotContainer c) {
    return c.achieveState(
        elbow.orElse(null),
        elevator.orElse(null),
        intakePistons.orElse(null),
        intakeSpeed.orElse(null),
        intakeBrake.orElse(null),
        true,
        0.0);
  }
}
