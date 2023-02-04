package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class MovArm extends CommandBase {
  private Arm arm;
  private DoubleSupplier speed;

  public MovArm(Arm arm, DoubleSupplier speed) {
    this.arm = arm;
    this.speed = speed;
    addRequirements(arm);
  }

  public MovArm(Arm arm, double speed) {
    this(arm, () -> speed);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    arm.armMove(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) { 
    arm.armMove(Constants.Arm.stopSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}