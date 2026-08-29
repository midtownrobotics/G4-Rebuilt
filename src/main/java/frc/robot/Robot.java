package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRoller;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.util.TunerConstants;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final Drivetrain m_drivetrain;
  private final IntakePivot m_intakePivot;
  private final IntakeRoller m_intakeRoller;
  private final Hood m_hood;
  private final Flywheel m_flywheel;
  private final Feeder m_feeder;

  public Robot() {
    Logger.recordMetadata("ProjectName", "G4-Rebuilt");
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    DriverStation.silenceJoystickConnectionWarning(isSimulation());

    m_drivetrain = new Drivetrain(
        new GyroIOSim(),
        new ModuleIOSim(TunerConstants.FrontLeft),
        new ModuleIOSim(TunerConstants.FrontRight),
        new ModuleIOSim(TunerConstants.BackLeft),
        new ModuleIOSim(TunerConstants.BackRight));

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
      m_drivetrain.runVelocity(new ChassisSpeeds(
          x * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
          y * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
          omega * m_drivetrain.getMaxAngularSpeedRadPerSec()));
    }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
