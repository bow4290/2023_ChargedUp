package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elbow extends SubsystemBase {
  private final TalonFX elbowPivot;
  private final TalonFX elbowPivot2;

  public double degreesToTicks(double degrees) {
    return degrees * Constants.Arm.ticksPerDegree;
  }

  public Elbow() {
    elbowPivot = new TalonFX(Constants.Arm.armPivotID);
    elbowPivot.configFactoryDefault();
    elbowPivot.configForwardSoftLimitEnable(true);
    elbowPivot.configForwardSoftLimitThreshold(degreesToTicks(95));
    elbowPivot.configReverseSoftLimitEnable(true);
    elbowPivot.configReverseSoftLimitThreshold(degreesToTicks(-95));

    // TODO Needs to be tuned for follower motors
    elbowPivot.config_kP(0, 1);
    elbowPivot.config_kD(0, 10);
    elbowPivot.config_kF(0, 0.0565);
    elbowPivot.configMotionAcceleration(8000);
    elbowPivot.configMotionCruiseVelocity(10000);

    elbowPivot.setStatusFramePeriod(
        StatusFrame.Status_1_General, 5); // This might make following more accurate?

    elbowPivot.setInverted(true);
    elbowPivot.setNeutralMode(NeutralMode.Brake);

    elbowPivot2 = new TalonFX(Constants.Arm.armPivot2ID);
    elbowPivot2.configFactoryDefault();
    elbowPivot2.follow(elbowPivot);
    elbowPivot2.setInverted(
        InvertType.OpposeMaster); // One motor needs to be CCW, the other needs to be CW
    // "Motor controllers that are followers can have slower update rates for this group without
    // impacting performance."
    // - Phoenix docs
    elbowPivot2.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    elbowPivot2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

    SmartDashboard.putData("Arm", this);

    setDefaultCommand(retainPositionCmd());
  }

  // SmartDashboard stuff
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Power", elbowPivot::getMotorOutputPercent, null);
    builder.addStringProperty("Mode", () -> elbowPivot.getControlMode().toString(), null);
    builder.addDoubleProperty("Target Pos", elbowPivot::getActiveTrajectoryPosition, null);
    builder.addDoubleProperty("Pos", this::getPosition, null);
    builder.addDoubleProperty("Target Vel", elbowPivot::getActiveTrajectoryVelocity, null);
    builder.addDoubleProperty("Vel", elbowPivot::getSelectedSensorVelocity, null);
    builder.addDoubleProperty("Degrees", () -> getPosition() * Constants.Arm.degreesPerTick, null);
  }

  public double getPosition() {
    return elbowPivot.getSelectedSensorPosition();
  }

  public void move(double speed) {
    elbowPivot.set(ControlMode.PercentOutput, speed);
  }

  public void pos(double pos) {
    elbowPivot.set(ControlMode.MotionMagic, pos);
  }

  public void retainPosition() {
    move(0); // THIS IS ONLY FOR TEST PURPOSES TO SEE IF BOTH FALCONS CAN BRAKE
    // armPivot.set(ControlMode.MotionMagic, getPosition());
  }

  public Command retainPositionCmd() {
    // NOTE: CHANGE THIS TO A startEnd IF YOU ARE USING MOTION MAGIC FOR THIS
    return runEnd(this::retainPosition, () -> move(0));
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

  public void resetToZero() {
    elbowPivot.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {}
}
