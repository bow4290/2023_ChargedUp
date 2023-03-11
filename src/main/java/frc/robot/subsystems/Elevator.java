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

  public void position(double pos) {
    mmPosition = pos;
    elevatorMotor.set(ControlMode.MotionMagic, pos);
  }

  private double mmPosition;

  public Command positionCmd(double pos) {
    return startEnd(() -> position(pos), () -> {})
        .beforeStarting(() -> mmPosition = pos)
        .until(this::isFinished)
        .withTimeout(Constants.Elevator.autoTimeout);
  }

  public boolean isFinished() {
    return (Math.abs(getPosition() - mmPosition) < Constants.Elevator.positionEps)
        && (Math.abs(elevatorMotor.getSelectedSensorVelocity()) < Constants.Elevator.velocityEps);
  }

  public Command goToBase() {
    return positionCmd(Constants.Elevator.base);
  }

  public Command goToMid() {
    return positionCmd(Constants.Elevator.middle);
  }

  public Command goToMax() {
    return positionCmd(Constants.Elevator.max);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }
}
