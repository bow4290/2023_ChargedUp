package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  private final TalonFX armPivot;
  private final TalonFX armPivot2;

  public double degreesToTicks(double degrees) {
    return degrees * Constants.Arm.ticksPerDegree;
  }

  public Arm() {
    armPivot = new TalonFX(Constants.Arm.armPivotID);
    armPivot.configFactoryDefault();
    armPivot.configForwardSoftLimitEnable(false);
    armPivot.configForwardSoftLimitThreshold(degreesToTicks(90));
    armPivot.configReverseSoftLimitEnable(false);
    armPivot.configReverseSoftLimitThreshold(degreesToTicks(-90));
    // Needs to be tuned for follower motors
    armPivot.config_kP(0, 1);
    armPivot.config_kD(0, 10);
    armPivot.config_kF(0, 0.0565);
    armPivot.configMotionAcceleration(6000);
    armPivot.configMotionCruiseVelocity(10000);

    armPivot.setStatusFramePeriod(StatusFrame.Status_1_General,5); // This might make following more accurate?

    armPivot.setInverted(true);
    armPivot.setNeutralMode(NeutralMode.Brake);
    
    armPivot2 = new TalonFX(Constants.Arm.armPivot2ID);
    armPivot2.configFactoryDefault();
    armPivot2.follow(armPivot);
    armPivot2.setInverted(InvertType.OpposeMaster); // One motor needs to be CCW, the other needs to be CW
    // "Motor controllers that are followers can have slower update rates for this group without impacting performance."
    // - Phoenix docs
    armPivot2.setStatusFramePeriod(StatusFrame.Status_1_General,100);
    armPivot2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,100);
  }

  public double getPosition() {
    return armPivot.getSelectedSensorPosition();
  }

  private double lastPowerSet = 0;

  public void move(double speed) {
    lastPowerSet = speed;
    armPivot.set(ControlMode.PercentOutput, speed);
  }

  public void pos(double pos) {
    // lastPowerSet = speed;
    // System.out.println("Position set");
    armPivot.set(ControlMode.MotionMagic, pos);
  }

  public Command moveCmd(DoubleSupplier speed) {
    return this.runEnd(() -> move(speed.getAsDouble()), () -> move(0));
  }

  public Command posCmd(double position) {
    return this.runEnd(() -> pos(position), () -> armPivot.set(ControlMode.PercentOutput, 0));
  }

  public Command moveWhileWantedCommand(DoubleSupplier speed) {
    return this.runEnd(() -> move(speed.getAsDouble()), () -> move(0))
            .until(() -> Math.abs(speed.getAsDouble()) < 0.075);
  }

  public Command holdPositionWhileNotWantedCommand(DoubleSupplier speed) {
    return this.startEnd(() -> pos(getPosition()), () -> move(0))
            .until(() -> Math.abs(speed.getAsDouble()) >= 0.075);
  }

  public Command moveAndorHoldCommand(DoubleSupplier speed) {
    return moveWhileWantedCommand(speed).andThen(holdPositionWhileNotWantedCommand(speed)).repeatedly();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Power: ", lastPowerSet);
  }
}
