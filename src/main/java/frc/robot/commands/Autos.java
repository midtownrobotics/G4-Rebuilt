// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.LoggedTunableNumber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakePivot;

public final class Autos {

  private final AutoFactory m_factory;
  private final Drive m_drive;
  private final IntakePivot m_intake;
  private final LoggedTunableNumber m_hubSwipeDelaySeconds = new LoggedTunableNumber("HubSwipeDelaySeconds", 0.0);

  public Autos(Drive drive, IntakePivot intake) {
    m_drive = drive;
    m_intake = intake;
    m_factory = new AutoFactory(m_drive::getPose, m_drive::resetPose, m_drive::followPath, true, m_drive);
  }

  public AutoRoutine MadtownLeft() {
    AutoRoutine routine = m_factory.newRoutine("MadtownLeft");
    AutoTrajectory TrenchSweep = routine.trajectory("TrenchSweep").mirrorY();
    AutoTrajectory TrenchSweep2 = routine.trajectory("TrenchSweep").mirrorY();
    AutoTrajectory BackwardsBump = routine.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory BackwardsBump2 = routine.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory BumpToTrenchSOTM = routine.trajectory("BumpToTrenchSOTM").mirrorY();

    TrenchSweep.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    TrenchSweep.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    TrenchSweep.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    TrenchSweep.done().onTrue(BackwardsBump.cmd());

    BackwardsBump.done().onTrue(BumpToTrenchSOTM.cmd());

    // BumpToTrenchSOTM.active().onTrue(/* start shooting */);
    // BumpToTrenchSOTM.atTime("stopshoot").onTrue(/* stop shooting */);
    BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

    TrenchSweep2.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    TrenchSweep2.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

    // BackwardsBump2.done().onTrue(/* start shooting */);

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

    TrenchSweep.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    TrenchSweep.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    TrenchSweep.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    TrenchSweep.done().onTrue(BackwardsBump.cmd());

    BackwardsBump.done().onTrue(BumpToTrenchSOTM.cmd());

    // BumpToTrenchSOTM.active().onTrue(/* start shooting */);
    // BumpToTrenchSOTM.atTime("stopshoot").onTrue(/* stop shooting */);
    BumpToTrenchSOTM.done().onTrue(TrenchSweep2.cmd());

    TrenchSweep2.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    TrenchSweep2.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    TrenchSweep2.done().onTrue(BackwardsBump2.cmd());

    // BackwardsBump2.done().onTrue(/* start shooting */);

    routine.active().onTrue(
        Commands.sequence(
            TrenchSweep.resetOdometry(),
            TrenchSweep.cmd()));
    return routine;
  }

  public AutoRoutine HubSwipeLeft() {
    AutoRoutine routine = m_factory.newRoutine("HubSwipeLeft");
    AutoTrajectory HubSwipe = routine.trajectory("HubSwipe").mirrorY();

    HubSwipe.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING).asProxy());
    HubSwipe.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    HubSwipe.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    // HubSwipe.done().onTrue(/* start shooting */);

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

    HubSwipe.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING).asProxy());
    HubSwipe.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    HubSwipe.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    // HubSwipe.done().onTrue(/* start shooting */);

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

    copy1002left.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING).asProxy());
    copy1002left.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    copy1002left.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    // copy1002left.done().onTrue(/* start shooting */);
    copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

    // trenchLineUp1002.active().onTrue(/* stop shooting */);
    trenchLineUp1002.done().onTrue(copy1002left2.cmd());

    copy1002left2.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING).asProxy());
    copy1002left2.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    copy1002left2.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    // copy1002left2.done().onTrue(/* start shooting */);

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

    copy1002left.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING).asProxy());
    copy1002left.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    copy1002left.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    // copy1002left.done().onTrue(/* start shooting */);
    copy1002left.doneDelayed(5).onTrue(trenchLineUp1002.cmd());

    // trenchLineUp1002.active().onTrue(/* stop shooting */);
    trenchLineUp1002.done().onTrue(copy1002left2.cmd());

    copy1002left2.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING).asProxy());
    copy1002left2.atTime("startintake").onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    copy1002left2.atTime("stopintake").onTrue(m_intake.setStateCommand(IntakePivot.State.NOT_INTAKING));
    // copy1002left2.done().onTrue(/* start shooting */);

    routine.active().onTrue(
        Commands.sequence(
            copy1002left.resetOdometry(),
            copy1002left.cmd()));
    return routine;
  }

  public AutoRoutine match13Depot() {
    AutoRoutine routine = m_factory.newRoutine("match13Depot");
    AutoTrajectory CenterDepot = routine.trajectory("CenterDepot");

    CenterDepot.active().onTrue(m_intake.setStateCommand(IntakePivot.State.INTAKING));
    // CenterDepot.active().onTrue(/* rev shooter */);
    // CenterDepot.done().onTrue(/* start shooting */);
    CenterDepot.doneDelayed(8);

    routine.active().onTrue(
        Commands.sequence(
            CenterDepot.resetOdometry(),
            CenterDepot.cmd()));
    return routine;
  }

  public AutoRoutine rightHubCleanUp() {
    AutoRoutine rightHubCleanUp = m_factory.newRoutine("rightHubCleanUp");
    AutoTrajectory RightTrenchToCenterBack = rightHubCleanUp.trajectory("RightTrenchToCenterBack");
    AutoTrajectory RightHubCleanup = rightHubCleanUp.trajectory("RightHubCleanup");
    AutoTrajectory BackwardsBump = rightHubCleanUp.trajectory("BackwardsBump").mirrorY();
    AutoTrajectory LeftBumpToDepot = rightHubCleanUp.trajectory("LeftBumpToDepot");

    RightTrenchToCenterBack.done().onTrue(RightHubCleanup.cmd());
    RightHubCleanup.done().onTrue(BackwardsBump.cmd());
    // BackwardsBump.atTime(0.7).onTrue(/* rev shooter */);
    BackwardsBump.done().onTrue(LeftBumpToDepot.cmd());
    // LeftBumpToDepot.active().onTrue(/* start shooting */);
    // LeftBumpToDepot.atTime("stopShooting").onTrue(m_robotCommands.fill()); TODO
    // LeftBumpToDepot.atTime("startShooting").onTrue(/* start shooting */);

    rightHubCleanUp.active().onTrue(
        Commands.sequence(
            m_intake.setStateCommand(IntakePivot.State.INTAKING).asProxy().withTimeout(Seconds.of(2)),
            RightTrenchToCenterBack.resetOdometry(),
            RightTrenchToCenterBack.cmd()));
    return rightHubCleanUp;
  }
}
