package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics simulation for one swerve module, based on G3-2026 event-cmp. */
public class ModuleIOSim implements ModuleIO {
  private static final double kDriveKvRotations = 0.91035;
  private static final double kDriveKv =
      1.0 / Units.rotationsToRadians(1.0 / kDriveKvRotations);
  private static final DCMotor kDriveGearbox = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor kTurnGearbox = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_turnSim;
  private final PIDController m_driveController = new PIDController(0.05, 0.0, 0.0);
  private final PIDController m_turnController = new PIDController(8.0, 0.0, 0.0);

  private boolean m_driveClosedLoop;
  private boolean m_turnClosedLoop;
  private double m_driveFeedforwardVolts;
  private double m_driveAppliedVolts;
  private double m_turnAppliedVolts;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    m_driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            kDriveGearbox, constants.DriveInertia, constants.DriveMotorGearRatio),
        kDriveGearbox);
    m_turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            kTurnGearbox, constants.SteerInertia, constants.SteerMotorGearRatio),
        kTurnGearbox);
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (m_driveClosedLoop) {
      m_driveAppliedVolts = m_driveFeedforwardVolts
          + m_driveController.calculate(m_driveSim.getAngularVelocityRadPerSec());
    } else {
      m_driveController.reset();
    }
    if (m_turnClosedLoop) {
      m_turnAppliedVolts = m_turnController.calculate(m_turnSim.getAngularPositionRad());
    } else {
      m_turnController.reset();
    }

    m_driveAppliedVolts = MathUtil.clamp(m_driveAppliedVolts, -12.0, 12.0);
    m_turnAppliedVolts = MathUtil.clamp(m_turnAppliedVolts, -12.0, 12.0);
    m_driveSim.setInputVoltage(m_driveAppliedVolts);
    m_turnSim.setInputVoltage(m_turnAppliedVolts);
    m_driveSim.update(0.02);
    m_turnSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps());
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    m_driveClosedLoop = false;
    m_driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    m_turnClosedLoop = false;
    m_turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveClosedLoop = true;
    m_driveFeedforwardVolts = kDriveKv * velocityRadPerSec;
    m_driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    m_turnClosedLoop = true;
    m_turnController.setSetpoint(rotation.getRadians());
  }
}
