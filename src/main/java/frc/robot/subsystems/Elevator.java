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
    elevatorMotor.config_kP(0, 0.3);
    elevatorMotor.config_kF(0, 0.058);
    elevatorMotor.config_kD(0, 2);
    elevatorMotor.configMotionAcceleration(10000);
    elevatorMotor.configMotionCruiseVelocity(10000);
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
    mmPosition = position;
    elevatorMotor.set(ControlMode.MotionMagic, position);
  }

  private double mmPosition;

  public Command positionCmd(double posi) {
    return run(() -> pos(posi))
        .beforeStarting(() -> mmPosition = posi)
        .until(
            () ->
                (Math.abs(getPosition() - mmPosition) < Constants.Elevator.positionEps)
                    && (Math.abs(elevatorMotor.getSelectedSensorVelocity())
                        < Constants.Elevator.velocityEps))
        .withTimeout(2.5);
  }

  public Command positionBaseCmd() {
    return positionCmd(Constants.Elevator.base);
  }

  public Command positionMidCmd() {
    return positionCmd(Constants.Elevator.middle);
  }

  public Command positionMaxCmd() {
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
  public void periodic() {}
}
