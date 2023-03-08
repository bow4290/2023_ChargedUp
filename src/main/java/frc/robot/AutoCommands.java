package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Command topCone() {
        return Commands.sequence(
                s_Elevator.positionMaxCmd(),
                s_Elbow.posDegCmd(45),
                s_Intake.pistonsCubeCmd(),
                s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd()),
                s_Intake.pistonsConeCmd());
    }

    public Command topCube() {
        return Commands.sequence(
                s_Elevator.positionMaxCmd(),
                s_Elbow.posDegCmd(45),
                s_Intake.spinEjectCmd().withTimeout(0.5),
                s_Elbow.posDegCmd(0).alongWith(s_Elevator.positionBaseCmd()));
    }
}
