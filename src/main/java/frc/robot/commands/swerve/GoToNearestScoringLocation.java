package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Adapted from WPILib SwerveControllerCommand. Generates trajectory to scoring location once ran
 */
public class GoToNearestScoringLocation extends CommandBase {
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private Supplier<Rotation2d> m_desiredRotation;

  public GoToNearestScoringLocation(Swerve s_Swerve) {
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    var pxController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    var pyController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    m_controller = new HolonomicDriveController(pxController, pyController, thetaController);
    m_outputModuleStates = s_Swerve::setModuleStates;
    m_pose = s_Swerve::getPose;
    m_kinematics = Constants.Swerve.swerveKinematics;
  }

  Pose2d startPos;
  Pose2d finalPos;

  private void genTrajectory() {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    m_trajectory = TrajectoryGenerator.generateTrajectory(startPos, List.of(), finalPos, config);
    m_desiredRotation =
        () ->
            m_trajectory
                .getStates()
                .get(m_trajectory.getStates().size() - 1)
                .poseMeters
                .getRotation();
  }

  @Override
  public void initialize() {
    startPos = m_pose.get();
    // Find the final position based on stuff
    // note: these are approximations, todo get the exact values soon
    var nearest = new double[] {0.5, 1.07, 1.63, 2.19, 2.75, 3.31, 3.86, 4.42, 4.98};

    double currentY = startPos.getY();
    double selected = -100;
    double highestDeviation = 1000;
    // Try to select thing
    for (double value : nearest) {
      if (Math.abs(value - currentY) < highestDeviation) {
        selected = value;
        highestDeviation = Math.abs(value - currentY);
      }
    }

    Rotation2d thing2;

    if (Math.abs(startPos.getRotation().getDegrees()) < 180) {
      thing2 = new Rotation2d(0);
    } else {
      thing2 = Rotation2d.fromDegrees(180);
    }

    finalPos = new Pose2d(new Translation2d(startPos.getX(), selected), thing2);
    genTrajectory();

    m_timer.restart();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
