package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controls.Controls;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;

/** Builds the reusable commands used by G4 controls and autonomous routines. */
public class RobotCommands {
  private final Controls m_controls;
  private final Drive m_drive;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Hood m_hood;
  private final Flywheel m_flywheel;
  private final Feeder m_feeder;

  public RobotCommands(
      Controls controls,
      Drive drive,
      IntakePivot intakePivot,
      IntakeRoller intakeRoller,
      Hood hood,
      Flywheel flywheel,
      Feeder feeder) {
    m_controls = controls;
    m_drive = drive;
    m_intakePivot = intakePivot;
    m_intakeRoller = intakeRoller;
    m_hood = hood;
    m_flywheel = flywheel;
    m_feeder = feeder;
  }

  public Command driveCommand() {
    return m_drive
        .run(
            () -> {
              ChassisSpeeds fieldRelativeSpeeds =
                  new ChassisSpeeds(
                      m_controls.getDriveForward() * m_drive.getMaxLinearSpeedMetersPerSec(),
                      m_controls.getDriveLeft() * m_drive.getMaxLinearSpeedMetersPerSec(),
                      m_controls.getDriveRotation() * m_drive.getMaxAngularSpeedRadPerSec());
              m_drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      fieldRelativeSpeeds, m_drive.getRotation()));
            })
        .withName("driveCommand");
  }

  public Command idle() {
    return Commands.parallel(
            m_intakePivot.setStateCommand(IntakePivot.State.NOT_INTAKING),
            m_intakeRoller.stopCommand(),
            m_hood.setMinimumAngleCommand(),
            m_flywheel.stopCommand(),
            m_feeder.stopCommand())
        .withName("idle");
  }

  public Command runIntake() {
    return Commands.parallel(
            m_intakePivot.setStateCommand(IntakePivot.State.INTAKING),
            m_intakeRoller.setVoltageCommand(Volts.of(12.0)))
        .finallyDo(
            () -> {
              m_intakePivot.setState(IntakePivot.State.NOT_INTAKING);
              m_intakeRoller.stop();
            })
        .withName("runIntake");
  }

  public Command shoot() {
    return Commands.parallel(
            m_flywheel.setVoltageCommand(Volts.of(12.0)), m_feeder.runForwardCommand())
        .finallyDo(
            () -> {
              m_flywheel.stop();
              m_feeder.stop();
            })
        .withName("shoot");
  }

  public Command unjam() {
    return Commands.parallel(
            m_intakeRoller.setVoltageCommand(Volts.of(-12.0)), m_feeder.runReverseCommand())
        .finallyDo(
            () -> {
              m_intakeRoller.stop();
              m_feeder.stop();
            })
        .withName("unjam");
  }

  public Command feedFuel() {
    return m_feeder.runForwardCommand().withName("feedFuel");
  }

  public Command increaseHoodAngle() {
    return m_hood.adjustAngleCommand(Hood.kAdjustmentStep).withName("increaseHoodAngle");
  }

  public Command decreaseHoodAngle() {
    return m_hood
        .adjustAngleCommand(Hood.kAdjustmentStep.unaryMinus())
        .withName("decreaseHoodAngle");
  }

  public Command zeroIntake() {
    return m_intakePivot
        .setStateCommand(IntakePivot.State.NOT_INTAKING)
        .withName("zeroIntake");
  }

  public Command zeroHood() {
    return m_hood.setMinimumAngleCommand().withName("zeroHood");
  }

  public Command disableShooting() {
    return Commands.parallel(m_flywheel.stopCommand(), m_feeder.stopCommand())
        .withName("disableShooting");
  }
}
