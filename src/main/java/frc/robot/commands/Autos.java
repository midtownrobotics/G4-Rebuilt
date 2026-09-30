// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.LoggedTunableNumber;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;

public final class Autos {

  private final AutoFactory m_factory;
  private final Drive m_drive;
  private final RobotCommands m_robotCommands;
  private final LoggedTunableNumber m_hubSwipeDelaySeconds = new LoggedTunableNumber("HubSwipeDelaySeconds", 0.0);
  private final FollowPath.Builder pathBuilder;

  public Autos(AutoFactory autoFactory, Drive drive, RobotCommands robotCommands) {
    m_drive = drive;
    m_factory = autoFactory;
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

  public AutoRoutine MadtownLeft() {
    AutoRoutine routine = m_factory.newRoutine("MadtownLeft");
    AutoTrajectory TrenchSweep = routine.trajectory("TrenchSweep").mirrorY();
    AutoTrajectory TrenchSweep2 = routine.trajectory("TrenchSweep").mirrorY();
    AutoTrajectory BackwardsBump = routine.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory BackwardsBump2 = routine.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory BumpToTrenchSOTM = routine.trajectory("BumpToTrench").mirrorY();

    TrenchSweep.active().onTrue(m_robotCommands.runIntake());
    TrenchSweep.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    TrenchSweep.done().onTrue(BackwardsBump.cmd());

    BackwardsBump.done().onTrue(m_robotCommands.shoot());
    BackwardsBump.doneDelayed(7.5).onTrue(BumpToTrenchSOTM.cmd().alongWith(m_robotCommands.stopShooting()));

    BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

    TrenchSweep2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep2.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

    BackwardsBump2.done().onTrue(m_robotCommands.shoot());

    routine.active().onTrue(
        Commands.sequence(
            TrenchSweep.resetOdometry(),
            TrenchSweep.cmd()));
    return routine;
  }

  public AutoRoutine MadtownRight() {
    AutoRoutine routine = m_factory.newRoutine("MadtownRight");
    AutoTrajectory TrenchSweep = routine.trajectory("TrenchSweep");
    AutoTrajectory TrenchSweep2 = routine.trajectory("TrenchSweep");
    AutoTrajectory BackwardsBump = routine.trajectory("BackwardsBump");
    AutoTrajectory BackwardsBump2 = routine.trajectory("BackwardsBump");
    AutoTrajectory BumpToTrenchSOTM = routine.trajectory("BumpToTrenchSOTM");

    TrenchSweep.active().onTrue(m_robotCommands.runIntake());
    TrenchSweep.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    TrenchSweep.done().onTrue(BackwardsBump.cmd());

    BackwardsBump.done().onTrue(BumpToTrenchSOTM.cmd());

    BumpToTrenchSOTM.active().onTrue(m_robotCommands.shoot().until(BumpToTrenchSOTM.atTime("stopshoot")));

    BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

    TrenchSweep2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    TrenchSweep2.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

    BackwardsBump2.done().onTrue(m_robotCommands.shoot());

    routine.active().onTrue(
        Commands.sequence(
            TrenchSweep.resetOdometry(),
            TrenchSweep.cmd()));
    return routine;
  }

  public AutoRoutine HubSwipeLeft() {
    AutoRoutine routine = m_factory.newRoutine("HubSwipeLeft");
    AutoTrajectory HubSwipe = routine.trajectory("HubSwipe").mirrorY();

    HubSwipe.active().onTrue(m_robotCommands.runIntake().asProxy());
    HubSwipe.atTime("startintake").onTrue(m_robotCommands.runIntake());
    HubSwipe.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    HubSwipe.done().onTrue(m_robotCommands.shoot());

    routine.active().onTrue(
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(m_hubSwipeDelaySeconds.get()), Set.of()),
            HubSwipe.resetOdometry(),
            HubSwipe.cmd()));
    return routine;
  }

  public AutoRoutine HubSwipeRight() {
    AutoRoutine routine = m_factory.newRoutine("HubSwipeRight");
    AutoTrajectory HubSwipe = routine.trajectory("HubSwipe");

    HubSwipe.active().onTrue(m_robotCommands.runIntake().asProxy());
    HubSwipe.atTime("startintake").onTrue(m_robotCommands.runIntake());
    HubSwipe.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    HubSwipe.done().onTrue(m_robotCommands.shoot());

    routine.active().onTrue(
        Commands.sequence(
            Commands.defer(() -> Commands.waitSeconds(m_hubSwipeDelaySeconds.get()), Set.of()),
            HubSwipe.resetOdometry(),
            HubSwipe.cmd()));
    return routine;

  }

  public AutoRoutine copy1002right() {
    AutoRoutine routine = m_factory.newRoutine("1002right");
    AutoTrajectory copy1002left = routine.trajectory("copy1002");
    AutoTrajectory copy1002left2 = routine.trajectory("copy1002");
    AutoTrajectory trenchLineUp1002 = routine.trajectory("trenchLineUp1002");

    copy1002left.active().onTrue(m_robotCommands.runIntake().asProxy());
    copy1002left.atTime("startintake").onTrue(m_robotCommands.runIntake());
    copy1002left.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    copy1002left.done().onTrue(m_robotCommands.shoot().until(trenchLineUp1002.active()));
    copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

    trenchLineUp1002.done().onTrue(copy1002left2.cmd());

    copy1002left2.active().onTrue(m_robotCommands.runIntake().asProxy());
    copy1002left2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    copy1002left2.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    copy1002left2.done().onTrue(m_robotCommands.shoot());

    routine.active().onTrue(
        Commands.sequence(
            copy1002left.resetOdometry(),
            copy1002left.cmd()));
    return routine;
  }

  public AutoRoutine copy1002left() {
    AutoRoutine routine = m_factory.newRoutine("1002left");
    AutoTrajectory copy1002left = routine.trajectory("copy1002").mirrorY();
    AutoTrajectory copy1002left2 = routine.trajectory("copy1002").mirrorY();
    AutoTrajectory trenchLineUp1002 = routine.trajectory("trenchLineUp1002").mirrorY();

    copy1002left.active().onTrue(m_robotCommands.runIntake().asProxy());
    copy1002left.atTime("startintake").onTrue(m_robotCommands.runIntake());
    copy1002left.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    copy1002left.done().onTrue(m_robotCommands.shoot().until(trenchLineUp1002.active()));
    copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

    trenchLineUp1002.done().onTrue(copy1002left2.cmd());

    copy1002left2.active().onTrue(m_robotCommands.runIntake().asProxy());
    copy1002left2.atTime("startintake").onTrue(m_robotCommands.runIntake());
    copy1002left2.atTime("stopintake").onTrue(m_robotCommands.zeroIntake());
    copy1002left2.done().onTrue(m_robotCommands.shoot());

    routine.active().onTrue(
        Commands.sequence(
            copy1002left.resetOdometry(),
            copy1002left.cmd()));
    return routine;
  }

  public AutoRoutine match13Depot() {
    AutoRoutine routine = m_factory.newRoutine("match13Depot");
    AutoTrajectory CenterDepot = routine.trajectory("CenterDepot");

    CenterDepot.active().onTrue(m_robotCommands.runIntake());
    CenterDepot.active().onTrue(m_robotCommands.revFlyweel());
    CenterDepot.done().onTrue(m_robotCommands.shoot());
    CenterDepot.doneDelayed(8);

    routine.active().onTrue(
        Commands.sequence(
            CenterDepot.resetOdometry(),
            CenterDepot.cmd()));
    return routine;
  }

  public AutoRoutine rightHubCleanUp() {
    AutoRoutine routine = m_factory.newRoutine("rightHubCleanUp");
    AutoTrajectory RightTrenchToCenterBack = routine.trajectory("RightTrenchToCenterBack");
    AutoTrajectory RightHubCleanup = routine.trajectory("RightHubCleanup");
    AutoTrajectory BackwardsBump = routine.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory LeftBumpToDepot = routine.trajectory("LeftBumpToDepot");

    RightTrenchToCenterBack.done().onTrue(RightHubCleanup.cmd());
    RightHubCleanup.done().onTrue(BackwardsBump.cmd());
    BackwardsBump.atTime(0.7).onTrue(m_robotCommands.revFlyweel());
    BackwardsBump.done().onTrue(LeftBumpToDepot.cmd());
    LeftBumpToDepot.active().onTrue(m_robotCommands.shoot().until(LeftBumpToDepot.atTime("stopShooting")));
    LeftBumpToDepot.atTime("startShooting").onTrue(m_robotCommands.shoot());

    routine.active().onTrue(
        Commands.sequence(
            m_robotCommands.runIntake().asProxy().withTimeout(Seconds.of(2)),
            RightTrenchToCenterBack.resetOdometry(),
            RightTrenchToCenterBack.cmd()));
    return routine;
  }
}
