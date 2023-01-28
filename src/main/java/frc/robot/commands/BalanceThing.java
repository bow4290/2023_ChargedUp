package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class BalanceThing extends PIDCommand {
    public BalanceThing(Swerve swerve) {
        super(
            new PIDController(-0.003, 0, 0),
            // Based on pitch
            swerve::getPitch,
            // We want to gain a pitch of 0
            0,
            // Output moves bot
            output -> swerve.drive(
                new Translation2d(output, 0.0), 0, false, true),
            swerve
        );
  
      // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
      // setpoint before it is considered as having reached the reference
      getController()
          .setTolerance(1, 1);
    }
  
    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
  }
  