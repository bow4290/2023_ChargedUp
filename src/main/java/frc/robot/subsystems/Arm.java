package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    armPivot.configForwardSoftLimitEnable(true);
    armPivot.configForwardSoftLimitThreshold(degreesToTicks(95));
    armPivot.configReverseSoftLimitEnable(true);
    armPivot.configReverseSoftLimitThreshold(degreesToTicks(-95));

    // TODO Needs to be tuned for follower motors
    armPivot.config_kP(0, 1);
    armPivot.config_kD(0, 10);
    armPivot.config_kF(0, 0.0565);
    armPivot.configMotionAcceleration(8000);
    armPivot.configMotionCruiseVelocity(10000);

    armPivot.setStatusFramePeriod(
        StatusFrame.Status_1_General, 5); // This might make following more accurate?

    armPivot.setInverted(true);
    armPivot.setNeutralMode(NeutralMode.Brake);

    armPivot2 = new TalonFX(Constants.Arm.armPivot2ID);
    armPivot2.configFactoryDefault();
    armPivot2.follow(armPivot);
    armPivot2.setInverted(
        InvertType.OpposeMaster); // One motor needs to be CCW, the other needs to be CW
    // "Motor controllers that are followers can have slower update rates for this group without
    // impacting performance."
    // - Phoenix docs
    armPivot2.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    armPivot2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

    SmartDashboard.putData("Arm", this);
  }

  // SmartDashboard stuff
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Power", armPivot::getMotorOutputPercent, null);
    builder.addStringProperty("Mode", () -> armPivot.getControlMode().toString(), null);
    builder.addDoubleProperty("Target Pos", armPivot::getActiveTrajectoryPosition, null);
    builder.addDoubleProperty("Pos", this::getPosition, null);
    builder.addDoubleProperty("Target Vel", armPivot::getActiveTrajectoryVelocity, null);
    builder.addDoubleProperty("Vel", armPivot::getSelectedSensorVelocity, null);
    builder.addDoubleProperty("Degrees", () -> getPosition() * Constants.Arm.degreesPerTick, null);
  }

  public double getPosition() {
    return armPivot.getSelectedSensorPosition();
  }

  public void move(double speed) {
    armPivot.set(ControlMode.PercentOutput, speed);
  }

  public void pos(double pos) {
    armPivot.set(ControlMode.MotionMagic, pos);
  }

  public void retainPosition() {
    move(0); // WARNING THIS IS ONLY FOR TEST PURPOSES
    // armPivot.set(ControlMode.MotionMagic, getPosition());
  }

  public Command retainPositionCmd() {
    return startEnd(this::retainPosition, () -> move(0));
  }

  public Command moveCmd(DoubleSupplier speed) {
    return runEnd(() -> move(speed.getAsDouble()), () -> move(0));
  }

  public Command posCmd(double position) {
    return startEnd(() -> pos(position), () -> move(0));
  }

  public Command posDegCmd(double positionDeg) {
    return posCmd(degreesToTicks(positionDeg));
  }

  // This is probably not the best way to do this, but it works:
  public Command moveWhileWantedCmd(DoubleSupplier speed) {
    return moveCmd(speed).until(() -> Math.abs(speed.getAsDouble()) < Constants.Arm.armDeadband);
  }

  public Command holdPositionWhileNotWantedCmd(DoubleSupplier speed) {
    return retainPositionCmd()
        .until(() -> Math.abs(speed.getAsDouble()) >= Constants.Arm.armDeadband);
  }

  public Command moveOrHoldCmd(DoubleSupplier speed) {
    return moveWhileWantedCmd(speed).andThen(holdPositionWhileNotWantedCmd(speed)).repeatedly();
  }

  public Command doubleMoveOrHoldCmd(DoubleSupplier backSpeed, DoubleSupplier frontSpeed) {
    return moveOrHoldCmd(
        () ->
            backSpeed.getAsDouble() * Constants.Arm.backSpeed
                + frontSpeed.getAsDouble() * Constants.Arm.frontSpeed);
  }

  @Override
  public void periodic() {}
}
