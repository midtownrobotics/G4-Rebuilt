package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.net.InetAddress;
import java.net.UnknownHostException;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.LoggedCommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;

public class Robot extends LoggedRobot {
  private static final CANBus kMechanismCANBus = new CANBus("rio");

  private Command m_autonomousCommand;

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final Drive m_drivetrain;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Hood m_hood;
  private final Flywheel m_flywheel;
  private final Feeder m_feeder;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    try {
      Logger.recordMetadata(
          "Hostname", InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
    } catch (UnknownHostException exception) {
      Logger.recordMetadata("Hostname", "Unknown");
    }

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    }
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    DriverStation.silenceJoystickConnectionWarning(isSimulation());

    if (RobotBase.isReal()) {
      m_drivetrain = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));
    } else {
      m_drivetrain = new Drive(
          new GyroIOSim(),
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));
    }

    // G3-2026 event-cmp mechanism CAN IDs.
    m_intakePivot = new IntakePivot(23, 25);
    m_intakeRoller = new IntakeRoller(24);
    m_hood = new Hood(26, 27);
    m_flywheel = new Flywheel(28, 29);
    m_feeder = new Feeder(34, 36);

    m_drivetrain.setDefaultCommand(m_drivetrain.run(() -> {
      double x = -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.1);
      double y = -MathUtil.applyDeadband(m_driverController.getLeftX(), 0.1);
      boolean usingKeyboardJoystick =
          RobotBase.isSimulation() && DriverStation.getStickAxisCount(0) <= 3;
      double rotationAxis = usingKeyboardJoystick
          ? m_driverController.getHID().getRawAxis(2)
          : m_driverController.getRightX();
      double omega = -MathUtil.applyDeadband(rotationAxis, 0.1);
      ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
          x * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
          y * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
          omega * m_drivetrain.getMaxAngularSpeedRadPerSec());
      m_drivetrain.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
          fieldRelativeSpeeds, m_drivetrain.getRotation()));
    }));

    var intakeTrigger = m_driverController.leftTrigger(0.2);
    intakeTrigger
        .onTrue(m_intakePivot.setStateCommand(IntakePivot.State.INTAKING))
        .whileTrue(m_intakeRoller.setVoltageCommand(Volts.of(12.0)))
        .onFalse(Commands.parallel(
            m_intakePivot.setStateCommand(IntakePivot.State.NOT_INTAKING),
            m_intakeRoller.stopCommand()));

    m_driverController.rightTrigger(0.2)
        .onTrue(m_hood.setMaximumAngleCommand())
        .onFalse(m_hood.setMinimumAngleCommand());

    SmartDashboard.putData(
        "StartSignalLogger", Commands.runOnce(SignalLogger::start).ignoringDisable(true));
    SmartDashboard.putData(
        "StopSignalLogger", Commands.runOnce(SignalLogger::stop).ignoringDisable(true));

    LoggedCommandScheduler.init(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Logger.recordOutput("Controls/LeftTrigger", m_driverController.getLeftTriggerAxis());
    Logger.recordOutput("Controls/RightTrigger", m_driverController.getRightTriggerAxis());
    Logger.recordOutput("CanBusUsage/Drive", TunerConstants.kCANBus.getStatus().BusUtilization);
    Logger.recordOutput("CanBusUsage/Mechs", kMechanismCANBus.getStatus().BusUtilization);
    Logger.recordOutput("matchTime", DriverStation.getMatchTime());
    Logger.recordOutput("RobotViz/RobotPose", m_drivetrain.getPose());

    double intakeAngleRadians = m_intakePivot.getAngle().in(Units.Radians);
    double hoodAngleRadians = m_hood.getAngleFromMinimum().in(Units.Radians);
    Logger.recordOutput("RobotViz/IntakeAngleRadians", intakeAngleRadians);
    Logger.recordOutput("RobotViz/HoodAngleRadians", hoodAngleRadians);

    // Both articulated mechanisms pivot about the robot's left-right (Y) axis.
    // Component order must match the asset files: model_0 is the intake and model_1 is the hood.
    Logger.recordOutput("RobotViz/G4ComponentPoses", new Pose3d[] {
        new Pose3d(0.2778252, 0.0, 0.1905,
            new Rotation3d(0.0, -intakeAngleRadians, 0.0)),
        new Pose3d(-0.2881376, 0.0, 0.4881626,
            new Rotation3d(0.0, -hoodAngleRadians, 0.0))
    });

    LoggedCommandScheduler.periodic();
  }

  @Override
  public void simulationInit() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
