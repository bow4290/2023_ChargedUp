package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elbow extends SubsystemBase {
  private final WPI_TalonFX elbowPivot;
  private final WPI_TalonFX elbowPivot2;
  private double mmPosition;
  private DoubleSupplier elevPos;

  public double degreesToTicks(double degrees) {
    return degrees * Constants.Elbow.ticksPerDegree;
  }

  public Elbow(DoubleSupplier elevPos) {
    this.elevPos = elevPos;
    SmartDashboard.putNumber("amongus_arm_ctrl", 0.0);
    elbowPivot = new WPI_TalonFX(Constants.Elbow.elbowPivotID);
    elbowPivot.configFactoryDefault();
    elbowPivot.configForwardSoftLimitEnable(true);
    elbowPivot.configForwardSoftLimitThreshold(degreesToTicks(Constants.Elbow.forwardLimit));
    elbowPivot.configReverseSoftLimitEnable(true);
    elbowPivot.configReverseSoftLimitThreshold(degreesToTicks(Constants.Elbow.backwardLimit));

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

    elbowPivot2 = new WPI_TalonFX(Constants.Elbow.elbowPivot2ID);
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
    elbowPivot.set(
        ControlMode.MotionMagic,
        pos,
        DemandType.ArbitraryFeedForward,
        calcAFFDeg(getPositionDegrees(), elevPos.getAsDouble()));
  }

  /// pos degrees, elev 0-1
  private double calcAFFDeg(double pos, double elev) {
    // 0 -> 90
    // -90 -> 0
    // 90 -> 180
    // 0 vertical is 90 in this address space
    double deg = pos + 90;
    double s = Math.cos(Math.toRadians(deg));
    // System.out.println("using aff of " + s * MathUtil.interpolate(0.05, 0.065, elev));
    // reduce to prevent rising, let proportional keep us up.
    return 0.75 * s * MathUtil.interpolate(0.05, 0.065, elev);
  }

  private double retainPositionGoal;

  public void retainPosition() {
    // 0.05 horiz elev in
    // 0.065 horiz elev out
    // System.out.println(SmartDashboard.getNumber("amongus_arm_ctrl", 0.0));
    // elbowPivot.set(ControlMode.PercentOutput, SmartDashboard.getNumber("amongus_arm_ctrl", 0.0));
    double pos = getPosition();
    if (Math.abs(retainPositionGoal - pos) * Constants.Elbow.degreesPerTick > 5.0) {
      retainPositionGoal = pos;
    }
    elbowPivot.set(
        ControlMode.MotionMagic,
        retainPositionGoal,
        DemandType.ArbitraryFeedForward,
        calcAFFDeg(pos * Constants.Elbow.degreesPerTick, elevPos.getAsDouble()));
  }

  public Command retainPositionCmd() {
    return new FunctionalCommand(
            () -> {
              move(0);
              retainPositionGoal = getPosition();
            },
            this::retainPosition,
            (Boolean a) -> move(0),
            () -> false,
            this)
        .beforeStarting(Commands.print("Retaining position"));
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
    return runEnd(
        () -> {
          mmPosition = pos;
          position(pos);
        },
        () -> {});
    // .beforeStarting(Commands.print("Setting position to: " + pos));
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

  public Command groundPosition() {
    return positionCmd(Constants.Elbow.Positions.ground);
  }

  public Command secondPosition() {
    return positionCmd(Constants.Elbow.Positions.second);
  }

  @Override
  public void periodic() {
    double p = getPosition();
    SmartDashboard.putNumber("elbow pos", p);
    SmartDashboard.putNumber("elbow pos deg est", p * Constants.Elbow.degreesPerTick);
  }
}
