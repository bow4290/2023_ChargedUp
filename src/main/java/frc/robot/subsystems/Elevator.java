package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
    elevatorMotor.setInverted(false);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  private double lastPowerSet = 0;

  public void move(double speed) {
    lastPowerSet = speed;
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public Command moveCmd(DoubleSupplier speed) {
    return this.runEnd(() -> move(speed.getAsDouble()), () -> move(0));
  }

  public Command moveCmd(double speed) {
    return moveCmd(() -> speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Power: ", lastPowerSet);
  }
}
