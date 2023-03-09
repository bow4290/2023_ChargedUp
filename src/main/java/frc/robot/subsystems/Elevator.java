package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMotor;

  public Elevator() {
    elevatorMotor = new TalonFX(Constants.Elevator.elevatorMotorID);

    elevatorMotor.configFactoryDefault();

    elevatorMotor.config_kP(0, Constants.Elevator.kP);
    elevatorMotor.config_kD(0, Constants.Elevator.kD);
    elevatorMotor.config_kF(0, Constants.Elevator.kF);

    elevatorMotor.configMotionAcceleration(Constants.Elevator.motionAcceleration);
    elevatorMotor.configMotionCruiseVelocity(Constants.Elevator.motionVelocity);
    elevatorMotor.configMotionSCurveStrength(Constants.Elevator.motionSmoothing);

    elevatorMotor.setInverted(false);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    setDefaultCommand(moveCmd(0));

    SmartDashboard.putData("Elevator", this);
  }

  // SmartDashboard stuff
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    /*
    builder.addDoubleProperty("Power", elevatorMotor::getMotorOutputPercent, null);
    builder.addStringProperty("Mode", () -> elevatorMotor.getControlMode().toString(), null);
    builder.addDoubleProperty("Target Pos", elevatorMotor::getActiveTrajectoryPosition, null);
    builder.addDoubleProperty("Pos", elevatorMotor::getSelectedSensorPosition, null);
    builder.addDoubleProperty("Target Vel", elevatorMotor::getActiveTrajectoryVelocity, null);
    builder.addDoubleProperty("Vel", elevatorMotor::getSelectedSensorVelocity, null);*/
  }

  public void move(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void pos(double position) {
    elevatorMotor.set(ControlMode.MotionMagic, position);
  }

  public Command posBaseAutoCmd() {
    return posAutoCmd(Constants.Elevator.base);
  }

  public Command posMidAutoCmd() {
    return posAutoCmd(Constants.Elevator.middle);
  }

  public Command posMaxAutoCmd() {
    return posAutoCmd(Constants.Elevator.max);
  }

  public Command posBaseManualCmd() {
    return posManualCmd(Constants.Elevator.base);
  }

  public Command posMidManualCmd() {
    return posManualCmd(Constants.Elevator.middle);
  }

  public Command posMaxManualCmd() {
    return posManualCmd(Constants.Elevator.max);
  }

  public Command moveCmd(DoubleSupplier speed) {
    return runEnd(() -> move(speed.getAsDouble()), () -> move(0));
  }

  public Command moveCmd(double speed) {
    return moveCmd(() -> speed);
  }

  public void resetToZero() {
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public double getPosition() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public void retainPosition() {
    // move(0); // This was for testing, to put falcons into coast
    elevatorMotor.set(ControlMode.MotionMagic, getPosition());
  }

  public Command retainPositionCmd() {
    return startEnd(
        () -> {
          move(0); // This is necessary, otherwise there are weird issues for some reason
          retainPosition();
        },
        () -> move(0));
  }

  public Command posManualCmd(double position) {
    return startEnd(() -> pos(position), this::retainPosition);
  }

  public Command posAutoCmd(double position) {
    return startEnd(() -> pos(position), () -> {})
        .until(this::atPosition)
        .withTimeout(Constants.Elevator.autoTimeout);
  }

  public Boolean atPosition() {
    return Math.abs(elevatorMotor.getClosedLoopError()) < Constants.Elevator.positionEps
        && Math.abs(elevatorMotor.getSelectedSensorVelocity()) < Constants.Elevator.velocityEps;
  }

  @Override
  public void periodic() {}
}
