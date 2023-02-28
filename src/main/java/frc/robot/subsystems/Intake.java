package frc.robot.subsystems;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.List;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntake;
  private final CANSparkMax rightIntake;
  private final DoubleSolenoid solenoid;

  public double lastPowerSet = 0;

  public enum IntakePistonStatus {
    Cone,
    Cube
  }

  public IntakePistonStatus status;

  private final SparkMaxPIDController leftPID;
  private final SparkMaxPIDController rightPID;

  private final RelativeEncoder leftPos;
  private final RelativeEncoder rightPos;

  public Intake() {
    leftIntake = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);
    rightIntake = new CANSparkMax(Constants.Intake.rightIntakeID, MotorType.kBrushless);

    leftPID = leftIntake.getPIDController();
    rightPID = rightIntake.getPIDController();

    leftPos = leftIntake.getEncoder();
    rightPos = rightIntake.getEncoder();

    List.of(leftIntake, rightIntake).forEach(motor -> {
      motor.restoreFactoryDefaults();
      motor.setInverted(false);
      motor.enableVoltageCompensation(11);
      motor.setIdleMode(IdleMode.kBrake);
      // Prevent smoking bot?
      motor.setSmartCurrentLimit(4);
    });

    List.of(leftPID, rightPID).forEach(pid -> {
      pid.setP(0.1); // Probably needs to be tuned or something!
      pid.setOutputRange(-0.3, 0.3);
    });

    solenoid =
        new DoubleSolenoid(
            Constants.Intake.pneumaticType,
            Constants.Intake.solenoidPortForward,
            Constants.Intake.solenoidPortReverse);

    setDefaultCommand(retainPositionCmd());
  }

  public void spin(double intakeSpeed) {
    lastPowerSet = intakeSpeed;
    leftIntake.set(intakeSpeed);
  }

  public void stayAtPosition() {
    leftPID.setReference(leftPos.getPosition(), CANSparkMax.ControlType.kPosition);
    rightPID.setReference(rightPos.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public Command retainPositionCmd() {
    return startEnd(this::stayAtPosition, this::stopSpinning);
  }

  public void stopSpinning() {
    spin(0);
  }

  public void setSolenoid(IntakePistonStatus status) {
    DoubleSolenoid.Value solenoidValue =
        status == IntakePistonStatus.Cone
            ? DoubleSolenoid.Value.kForward
            : DoubleSolenoid.Value.kReverse;
    this.status = status;

    solenoid.set(solenoidValue);
    System.out.println("SOLENOID " + solenoidValue.toString());
  }

  public Command pistonsCubeCmd() {
    return runOnce(() -> setSolenoid(IntakePistonStatus.Cube));
  }

  public Command pistonsConeCmd() {
    return runOnce(() -> setSolenoid(IntakePistonStatus.Cone));
  }

  public Command spinInCmd() {
    return runEnd(() -> spin(Constants.Intake.inSpeed), this::stopSpinning);
  }

  public Command spinEjectCmd() {
    return runEnd(() -> spin(Constants.Intake.ejectSpeed), this::stopSpinning);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Status: ", lastPowerSet);
  }
}
