package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakePistonStatus;
import frc.robot.Constants;

public class PistonTake extends CommandBase {
  private Intake intake;
  private IntakePistonStatus status;

  public PistonTake(Intake intake, IntakePistonStatus status) {
    this.intake = intake;
    this.status = status;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setSolenoids(status);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}