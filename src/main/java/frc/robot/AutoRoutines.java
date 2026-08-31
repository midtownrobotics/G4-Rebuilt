package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.GeometryUtil;
import frc.robot.commands.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;

/** Choreo autonomous routine definitions. Routines will be added as paths are developed. */
public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final RobotCommands m_robotCommands;
  private final FollowPath.Builder pathBuilder;
  private final Drive m_drive;

  private static final double kTrenchHeadingRad = 0.0;
  private static final Rotation2d kTrenchHeading =
      Rotation2d.fromRadians(kTrenchHeadingRad);
  private static final Rotation2d kTrenchHeadingMirrored =
      Rotation2d.fromRadians(-kTrenchHeadingRad);
  private static final Pose2d kTrenchEntryRight =
      new Pose2d(4.334, 0.585, kTrenchHeading);
  private static final Pose2d kTrenchExitRight = new Pose2d(6.589, 0.795, kTrenchHeading);

  public AutoRoutines(AutoFactory autoFactory, RobotCommands robotCommands, Drive drive) {
    m_autoFactory = autoFactory;
    m_robotCommands = robotCommands;
    m_drive = drive;

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            4.729,
            12.044,
            682.5,
            2945.6,
            Inches.of(1).in(Meters),
            2.0,
            0.3));

    pathBuilder =
        new FollowPath.Builder(
            drive,
            drive::getPose,
            drive::getChassisSpeeds,
            drive::runVelocity,
            new PIDController(7.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(4.0, 0.0, 0.0));
  }

  public Command driveToPose(Pose2d target) {
    return pathBuilder.build(new Path(new Path.Waypoint(target)));
  }

  public Command trenchSupport() {
    return Commands.defer(
        () -> {
          Pose2d current = m_drive.getPose();

          Pose2d entry = GeometryUtil.flip(kTrenchEntryRight);
          Pose2d exit = GeometryUtil.flip(kTrenchExitRight);

          Path path =
              new Path(
                  new PathConstraints()
                      .setMaxVelocityMetersPerSec(0.5)
                      .setMaxAccelerationMetersPerSec2(1.0),
                  new Path.Waypoint(current),
                  new Path.Waypoint(entry, 0.4),
                  new Path.Waypoint(exit));

          return pathBuilder.build(path);
        },
        Set.of(m_drive));
  }
}
