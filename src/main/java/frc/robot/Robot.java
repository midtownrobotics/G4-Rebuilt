package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.controls.Controls;
import frc.robot.controls.XboxControls;
import frc.lib.LoggedCommandScheduler;
import frc.robot.commands.RobotCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.HoodIOTalonFX;
import java.net.InetAddress;
import java.net.UnknownHostException;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static final CANBus kMechanismCANBus = new CANBus("rio");

  private Command m_autonomousCommand;

  private final Controls m_controls = new XboxControls(0);
  private final Drive m_drivetrain;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Hood m_hood;
  private final Flywheel m_flywheel;
  private final Feeder m_feeder;
  private final RobotCommands m_robotCommands;

  private final AutoFactory m_autoFactory;
  private final AutoRoutines m_autoRoutines;
  private final AutoChooser m_autoChooser;

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
      m_drivetrain =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
      m_intakePivot = new IntakePivot(new IntakePivotIOTalonFX(23, 25));
      m_intakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX(24));
      m_hood = new Hood(new HoodIOTalonFX(26, 27));
      m_flywheel = new Flywheel(new FlywheelIOTalonFX(28, 29));
      m_feeder = new Feeder(new FeederIOTalonFX(34, 36));
    } else {
      m_drivetrain =
          new Drive(
              new GyroIOSim(),
              new ModuleIOSim(TunerConstants.FrontLeft),
              new ModuleIOSim(TunerConstants.FrontRight),
              new ModuleIOSim(TunerConstants.BackLeft),
              new ModuleIOSim(TunerConstants.BackRight));
      m_intakePivot = new IntakePivot(new IntakePivotIOSim());
      m_intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
      m_hood = new Hood(new HoodIOSim());
      m_flywheel = new Flywheel(new FlywheelIOSim());
      m_feeder = new Feeder(new FeederIOSim());
    }

    // G3-2026 event-cmp mechanism CAN IDs.

    m_robotCommands =
        new RobotCommands(
            m_controls,
            m_drivetrain,
            m_intakePivot,
            m_intakeRoller,
            m_hood,
            m_flywheel,
            m_feeder);

    m_drivetrain.setDefaultCommand(m_robotCommands.driveCommand());

    configureBindings();

    m_autoFactory =
        new AutoFactory(
            m_drivetrain::getPose,
            m_drivetrain::resetPose,
            m_drivetrain::followPath,
            true,
            m_drivetrain);
    m_autoRoutines = new AutoRoutines(m_autoFactory, m_robotCommands, m_drivetrain);
    m_autoChooser = new AutoChooser("Do Nothing");

    generateAutoChooser();

    SmartDashboard.putData(
        "StartSignalLogger", Commands.runOnce(SignalLogger::start).ignoringDisable(true));
    SmartDashboard.putData(
        "StopSignalLogger", Commands.runOnce(SignalLogger::stop).ignoringDisable(true));

    LoggedCommandScheduler.init(CommandScheduler.getInstance());
  }

  private void generateAutoChooser() {
    // Add routines here once G4 autonomous paths are ready.
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    RobotModeTriggers.autonomous().whileTrue(m_autoChooser.selectedCommandScheduler());
  }

  private void configureBindings() {
    m_controls.idle().onTrue(m_robotCommands.idle());
    m_controls.intake().whileTrue(m_robotCommands.runIntake());
    m_controls.shoot().whileTrue(m_robotCommands.shoot());
    m_controls.unjam().whileTrue(m_robotCommands.unjam());
    m_controls.feedFuel().whileTrue(m_robotCommands.feedFuel());
    m_controls.increaseHoodAngle().onTrue(m_robotCommands.increaseHoodAngle());
    m_controls.decreaseHoodAngle().onTrue(m_robotCommands.decreaseHoodAngle());

    // G4 does not yet expose the G3 homing routines, so the matching chords return to the
    // known mechanism reference positions.
    m_controls.zeroIntake().onTrue(m_robotCommands.zeroIntake());
    m_controls.zeroHood().onTrue(m_robotCommands.zeroHood());
    m_controls.disableShooting().whileTrue(m_robotCommands.disableShooting());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

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
    Logger.recordOutput(
        "RobotViz/G4ComponentPoses",
        new Pose3d[] {
          new Pose3d(0.2778252, 0.0, 0.1905, new Rotation3d(0.0, -intakeAngleRadians, 0.0)),
          new Pose3d(-0.2881376, 0.0, 0.4881626, new Rotation3d(0.0, -hoodAngleRadians, 0.0))
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
