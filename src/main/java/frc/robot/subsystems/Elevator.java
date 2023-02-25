package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMotor;

  public Elevator() {
    elevatorMotor = new TalonFX(Constants.Elevator.elevatorMotorID);

    elevatorMotor.configFactoryDefault();
    elevatorMotor.config_kP(0, 1);
    elevatorMotor.config_kF(0, 0.073);
    elevatorMotor.configMotionAcceleration(10000);
    elevatorMotor.configMotionCruiseVelocity(10000);
    elevatorMotor.setInverted(false);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    setDefaultCommand(moveCmd(0));
  }

  public void move(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void pos(double position) {
    elevatorMotor.set(ControlMode.MotionMagic, position);
  }

  public Command positionCmd(double posi) {
    return run(() -> pos(posi));
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
    return this.runEnd(() -> move(speed.getAsDouble()), () -> move(0));
  }

  public Command moveCmd(double speed) {
    return moveCmd(() -> speed);
  }

  @Override
  public void periodic() {}
}
