package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.math.RateMeasurer;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AutoCommands {
  private Swerve s_Swerve;
  private Intake s_Intake;
  private Elbow s_Elbow;
  private Elevator s_Elevator;

  public AutoCommands(Swerve s_Swerve, Intake s_Intake, Elbow s_Elbow, Elevator s_Elevator) {
    this.s_Swerve = s_Swerve;
    this.s_Intake = s_Intake;
    this.s_Elbow = s_Elbow;
    this.s_Elevator = s_Elevator;
  }

  public Command baseArmAndElevator() {
    return s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd());
  }

  public Command topCone() {
    return Commands.sequence(
        s_Elevator.positionMaxCmd(),
        s_Elbow.posDegCmd(45),
        s_Intake.pistonsCubeCmd(),
        baseArmAndElevator(),
        s_Intake.pistonsConeCmd());
  }

  public Command topCube() {
    return Commands.sequence(
        s_Elevator.positionMaxCmd(),
        s_Elbow.posDegCmd(45),
        s_Intake.spinEjectCmd().withTimeout(0.5),
        baseArmAndElevator());
  }

  public Command attemptBalance() {
    RateMeasurer measurer = new RateMeasurer();
    return new FunctionalCommand(
        () -> measurer.init(s_Swerve.getTiltMagnitude()),
        () -> {
          double[] grav = s_Swerve.getGravity();
          double magnitude = Math.sqrt(grav[0] * grav[0] + grav[1] * grav[1]);
          Rotation2d rot = new Rotation2d(grav[0], grav[1]);
          measurer.addMeasurement(magnitude);
          SmartDashboard.putNumber("mag rate", measurer.getRate());
          if (magnitude > 0.05 || measurer.getRate() < 0.5) {
            s_Swerve.drive(new Translation2d(rot.getCos(), rot.getSin()), 0, false, true);
          } else {
            s_Swerve.lockModules();
          }
        },
        s_Swerve::lockModules,
        () -> false);
  }
}
