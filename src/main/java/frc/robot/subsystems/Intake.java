package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private final LinearFilter currentMeasurer = LinearFilter.movingAverage(8);

  public Intake() {
    leftIntake = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);
    rightIntake = new CANSparkMax(Constants.Intake.rightIntakeID, MotorType.kBrushless);

    leftPID = leftIntake.getPIDController();
    rightPID = rightIntake.getPIDController();

    leftPos = leftIntake.getEncoder();
    rightPos = rightIntake.getEncoder();

    List.of(leftIntake, rightIntake)
        .forEach(
            motor -> {
              motor.restoreFactoryDefaults();
              motor.setInverted(false);
              motor.enableVoltageCompensation(12);
              motor.setIdleMode(IdleMode.kBrake);
              // Prevent smoking bot?
              motor.setSmartCurrentLimit(20);
            });

    List.of(leftPID, rightPID)
        .forEach(
            pid -> {
              pid.setP(0.1); // Probably needs to be tuned or something!
              pid.setOutputRange(-0.2, 0.2);
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
    rightIntake.set(-intakeSpeed);
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
    System.out.println("Setting solenoid to " + solenoidValue);
  }

  public Command pistonsCubeCmd() {
    return runOnce(() -> setSolenoid(IntakePistonStatus.Cube));
  }

  public Command pistonsConeCmd() {
    return runOnce(() -> setSolenoid(IntakePistonStatus.Cone));
  }

  public Command spinInCmd() {
    return runEnd(() -> spin(Constants.Intake.inSpeed), this::retainPositionCmd);
  }

  public Command spinEjectCmd() {
    return runEnd(() -> spin(Constants.Intake.ejectSpeed), this::stopSpinning);
  }

  public Command autoEjectCmd() {
    return pistonsCubeCmd().andThen(spinEjectCmd().withTimeout(0.4));
  }

  private double currentFiltered;

  public Trigger intakeHasThing = new Trigger(() -> currentFiltered > 13).debounce(0.05);

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Status: ", lastPowerSet);
    double leftCurrent = leftIntake.getOutputCurrent();
    SmartDashboard.putNumber("Left Intake Current", leftCurrent);
    currentFiltered = currentMeasurer.calculate(leftCurrent);
    SmartDashboard.putNumber("Left Current Filtered", currentFiltered);

    SmartDashboard.putBoolean("Intake Has Thing", intakeHasThing.getAsBoolean());
  }
}
