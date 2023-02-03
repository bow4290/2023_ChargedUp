package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class SpInintake extends CommandBase {
  private Intake intake;
  private DoubleSupplier speed;

  public enum IntakeSpinStatus {
    Stop,
    Intake,
    Eject
  }

  public SpInintake(Intake intake, DoubleSupplier speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  public SpInintake(Intake intake, double speed) {
    this(intake, () -> speed);
  }

  public SpInintake(Intake intake, IntakeSpinStatus intakespin) {
    this(intake, 
        intakespin == IntakeSpinStatus.Stop  ?Constants.Intake.stopSpeed  : 
       intakespin == IntakeSpinStatus.Eject  ?Constants.Intake.ejectSpeed :
      intakespin == IntakeSpinStatus.Intake  ?Constants.Intake.inSpeed    :
    -4290); // Hopefully that never happens!
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.intakeSpin(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) { 
    intake.intakeSpin(Constants.Intake.stopSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}