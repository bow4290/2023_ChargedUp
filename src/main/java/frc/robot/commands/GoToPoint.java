package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.List;

public class GoToPoint extends SequentialCommandGroup {
  public GoToPoint(Swerve swerve, Pose2d from, Pose2d to) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(from, to),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);

    addCommands(swerveControllerCommand);
  }
}
