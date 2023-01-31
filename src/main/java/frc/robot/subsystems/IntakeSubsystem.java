package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor1;
  private final CANSparkMax intakeMotor2;


  public IntakeSubsystem() {
    intakeMotor1 = new CANSparkMax(2, MotorType.kBrushless);
    intakeMotor1.restoreFactoryDefaults();
    intakeMotor1.setInverted(false);
  //  intakeMotor1.enableVoltageCompensation(11);
  //  intakeMotor1.setOpenLoopRampRate(0.5);
    intakeMotor1.setIdleMode(IdleMode.kBrake);
    intakeMotor2 = new CANSparkMax(7, MotorType.kBrushless);
    intakeMotor2.restoreFactoryDefaults();
    intakeMotor2.setInverted(false);
    //intakeMotor2.enableVoltageCompensation(11);
   // intakeMotor2.setOpenLoopRampRate(0.5);
   intakeMotor2.setIdleMode(IdleMode.kBrake);

  }

  public void intakeSpin(double intakeSpeed) {
    intakeMotor1.set(intakeSpeed);
    intakeMotor2.set(-intakeSpeed);
  }

  public boolean isIntakeSpinning(){
    return Math.abs(intakeMotor1.get()) > 0 ? true : false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Status: ", intakeMotor1.get());
  }
}