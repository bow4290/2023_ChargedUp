package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;

public class Intake extends SubsystemBase {
  private final CANSparkMax intake;

  public double lastPowerSet = 0;

  private final SparkMaxPIDController PID;

  private final RelativeEncoder pos;

  public Intake() {
    intake = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);

    PID = intake.getPIDController();

    pos = intake.getEncoder();
    intake.restoreFactoryDefaults();
    intake.setInverted(false);
    intake.enableVoltageCompensation(11);
    intake.setIdleMode(IdleMode.kBrake);
    // Prevent smoking bot?
    intake.setSmartCurrentLimit(8);

    PID.setP(0.1); // Probably needs to be tuned or something!
    // Edit: it did not actually need to be tuned, it works decently well
    PID.setOutputRange(-0.3, 0.3);


    setDefaultCommand(retainPositionCmd());
  }

  public void spin(double intakeSpeed) {
    lastPowerSet = intakeSpeed;
    intake.set(intakeSpeed);
  }

  public void stayAtPosition() {
    PID.setReference(pos.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public Command retainPositionCmd() {
    return startEnd(this::stayAtPosition, this::stopSpinning);
  }

  public void stopSpinning() {
    spin(0);
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
