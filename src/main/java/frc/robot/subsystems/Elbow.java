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
  private double mmPosition;

  public double degreesToTicks(double degrees) {
    return degrees * Constants.Elbow.ticksPerDegree;
  }

  public Elbow() {
    elbowPivot = new TalonFX(Constants.Elbow.elbowPivotID);
    elbowPivot.configFactoryDefault();
    elbowPivot.configForwardSoftLimitEnable(false);
    // elbowPivot.configForwardSoftLimitThreshold(degreesToTicks(Constants.Elbow.forwardLimit));
    elbowPivot.configReverseSoftLimitEnable(false);
    // elbowPivot.configReverseSoftLimitThreshold(degreesToTicks(Constants.Elbow.backwardLimit));

    elbowPivot.config_kP(0, Constants.Elbow.kP);
    elbowPivot.config_kD(0, Constants.Elbow.kD);
    elbowPivot.config_kF(0, Constants.Elbow.kF);
    elbowPivot.configMotionCruiseVelocity(Constants.Elbow.motionVelocity);
    elbowPivot.configMotionAcceleration(Constants.Elbow.motionAcceleration);
    elbowPivot.configMotionSCurveStrength(Constants.Elbow.motionSmoothing);

    elbowPivot.setStatusFramePeriod(
        StatusFrame.Status_1_General, 5); // This might make following more accurate?

    elbowPivot.setInverted(true);
    elbowPivot.setNeutralMode(NeutralMode.Brake);

    elbowPivot2 = new TalonFX(Constants.Elbow.elbowPivot2ID);
    elbowPivot2.configFactoryDefault();
    elbowPivot2.follow(elbowPivot);
    elbowPivot2.setInverted(
        InvertType.OpposeMaster); // One motor needs to be CCW, the other needs to be CW
    // "Motor controllers that are followers can have slower update rates for this group without
    // impacting performance."
    // - Phoenix docs
    elbowPivot2.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    elbowPivot2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    elbowPivot2.setNeutralMode(NeutralMode.Brake);

    elbowPivot.configNeutralDeadband(0.02);

    SmartDashboard.putData("Arm", this);

    // Note that because of command groups this doesn't happen in auto
    setDefaultCommand(retainPositionCmd());
  }

  // SmartDashboard stuff
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // We need to figure out how to get this working again
    /* builder.addDoubleProperty("Power", elbowPivot::getMotorOutputPercent, null);
    builder.addStringProperty("Mode", () -> elbowPivot.getControlMode().toString(), null);
    builder.addDoubleProperty("Target Pos", elbowPivot::getActiveTrajectoryPosition, null);
    builder.addDoubleProperty("Pos", this::getPosition, null);
    builder.addDoubleProperty("Target Vel", elbowPivot::getActiveTrajectoryVelocity, null);
    builder.addDoubleProperty("Vel", elbowPivot::getSelectedSensorVelocity, null);
    builder.addDoubleProperty("Degrees", () -> getPosition() * Constants.Elbow.degreesPerTick, null);

    */
  }

  public double getPosition() {
    return elbowPivot.getSelectedSensorPosition();
  }

  public void move(double speed) {
    elbowPivot.set(ControlMode.PercentOutput, speed);
  }

  public void position(double pos) {
    mmPosition = pos;
    elbowPivot.set(ControlMode.MotionMagic, pos);
  }

  public void retainPosition() {
    move(0);
    elbowPivot.set(ControlMode.MotionMagic, getPosition());
  }

  public Command retainPositionCmd() {
    return startEnd(this::retainPosition, () -> move(0));
  }

  public Command moveCmd(DoubleSupplier speed) {
    return runEnd(() -> move(speed.getAsDouble()), () -> move(0));
  }

  public Command positionCmd(double pos) {
    return unendingPositionCmd(pos)
        .until(this::isFinished)
        .withTimeout(Constants.Elbow.autoTimeout);
  }

  public Command unendingPositionCmd(double pos) {
    return startEnd(() -> position(pos), () -> {}).beforeStarting(() -> mmPosition = pos);
    // .withTimeout(Constants.Elbow.autoTimeout);//whoops
  }

  public boolean isFinished() {
    return (Math.abs(getPosition() - mmPosition) < Constants.Elbow.rotationEps)
        && (Math.abs(elbowPivot.getSelectedSensorVelocity()) < Constants.Elbow.velocityEps);
  }

  public Command goToDeg(double deg) {
    return positionCmd(degreesToTicks(deg));
  }

  public Command goToDegUnending(double deg) {
    return unendingPositionCmd(degreesToTicks(deg));
  }

  public void resetToZero() {
    elbowPivot.setSelectedSensorPosition(0);
  }

  public double getPositionDegrees() {
    return getPosition() * Constants.Elbow.degreesPerTick;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elbow pos", getPositionDegrees());
  }
}
