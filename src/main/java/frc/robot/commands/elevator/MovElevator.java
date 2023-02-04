package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

public class MovElevator extends CommandBase {
  private Elevator elevator;
  private DoubleSupplier speed;

  public MovElevator(Elevator elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  public MovElevator(Elevator elevator, double speed) {
    this(elevator, () -> speed);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.elevatorMove(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) { 
    elevator.elevatorMove(Constants.Elevator.stopSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}