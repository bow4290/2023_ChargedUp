package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntake;
  private final CANSparkMax rightIntake;

  private final DoubleSolenoid leftSolenoid;
  private final DoubleSolenoid rightSolenoid;

  public double lastPowerSet = 0;

  public enum IntakePistonStatus {
    Cone,
    Cube
  }
  public IntakePistonStatus status;


  public Intake() {
    leftIntake = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);
    leftIntake.restoreFactoryDefaults();
    leftIntake.setInverted(false);
    leftIntake.enableVoltageCompensation(11);
    leftIntake.setIdleMode(IdleMode.kBrake);

    rightIntake = new CANSparkMax(Constants.Intake.rightIntakeID, MotorType.kBrushless);
    rightIntake.restoreFactoryDefaults();
    rightIntake.setInverted(false);
    rightIntake.enableVoltageCompensation(11);
    rightIntake.setIdleMode(IdleMode.kBrake);

    leftSolenoid = new DoubleSolenoid(Constants.Intake.pneumaticType, 
      Constants.Intake.leftSolenoidPortForward, Constants.Intake.leftSolenoidPortReverse);
    rightSolenoid = new DoubleSolenoid(Constants.Intake.pneumaticType, 
      Constants.Intake.rightSolenoidPortForward, Constants.Intake.rightSolenoidPortReverse);
  }

  public void spin(double intakeSpeed) {
    lastPowerSet = intakeSpeed;
    leftIntake.set(intakeSpeed);
    rightIntake.set(-intakeSpeed);
  }

  public void stopSpinning() {
    spin(0);
  }

  public void setSolenoids(IntakePistonStatus status) {
    DoubleSolenoid.Value solenoidValue = 
      status == IntakePistonStatus.Cone ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    this.status = status;

    leftSolenoid.set(solenoidValue);
    rightSolenoid.set(solenoidValue);
  }

  public Command pistonsCubeCmd () {
    return runOnce(() -> setSolenoids(IntakePistonStatus.Cube));
  }

  public Command pistonsConeCmd () {
    return runOnce(() -> setSolenoids(IntakePistonStatus.Cone));
  }

  public Command spinInCmd () { return startEnd(() -> spin(Constants.Intake.inSpeed), this::stopSpinning); }

  public Command spinEjectCmd () {
    return startEnd(() -> spin(Constants.Intake.ejectSpeed), this::stopSpinning);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Status: ", lastPowerSet);
  }
}