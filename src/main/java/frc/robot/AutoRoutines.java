package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;

public class AutoRoutines {
  private final AutoFactory m_autoFactory;
  private final RobotCommands m_robotCommands;
  private final FollowPath.Builder pathBuilder;
  private final Drive m_drive;

  private static final double kTrenchHeadingRad = 0.0;
  private static final Rotation2d kTrenchHeading = Rotation2d.fromRadians(kTrenchHeadingRad);
  private static final Rotation2d kTrenchHeadingMirrored = Rotation2d.fromRadians(-kTrenchHeadingRad);
  private static final Pose2d kTrenchEntryRight = new Pose2d(4.334, 0.585, kTrenchHeading);
  private static final Pose2d kTrenchExitRight = new Pose2d(6.589, 0.795, kTrenchHeading);

  public AutoRoutines(AutoFactory autoFactory, Drive drive, RobotCommands robotCommands) {
    m_autoFactory = autoFactory;
    m_drive = drive;
    m_robotCommands = robotCommands;

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(
            4.729,
            12.044,
            682.5,
            2945.6,
            Inches.of(1).in(Meters),
            2.0,
            0.3));

    pathBuilder = new FollowPath.Builder(
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

  public AutoRoutine test() {
    AutoRoutine routine = m_autoFactory.newRoutine("test");
    AutoTrajectory testRoutine = routine.trajectory("test");

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.runIntake().asProxy().withTimeout(Seconds.of(2)),
            testRoutine.resetOdometry(),
            testRoutine.cmd()));
    return routine;
  }
}
