package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;

public class Hood extends SubsystemBase {
  private static final double kRotorToHoodRatio = 62.0 / 14.0;

  private static final Angle kMinimumAngle = Degrees.of(16.25);
  private static final Angle kMaximumAngle = Degrees.of(35.0);

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("Hood/kP", 800.0);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("Hood/kI", 0.0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("Hood/kD", 18.0);
  private final LoggedTunableNumber m_kS = new LoggedTunableNumber("Hood/kS", 0.395);
  private final LoggedTunableNumber m_kV = new LoggedTunableNumber("Hood/kV", 30.0);
  private final LoggedTunableNumber m_kG = new LoggedTunableNumber("Hood/kG", 0.015);

  private final TalonFX m_motor;
  private final CANcoder m_encoder;

  private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private final LinearFilter m_currentSpikeFilter = LinearFilter.movingAverage(5);
  private final Trigger m_currentSpikeTrigger;
  private final Trigger m_isNearSetpointTrigger;

  private final Alert m_motorConnectionAlert = new Alert(
      "Hood TalonFX motor is not connected", AlertType.kWarning);
  private final Alert m_encoderConnectionAlert = new Alert(
      "Hood CANcoder is not connected", AlertType.kWarning);
  private final Alert m_stallAlert = new Alert("Hood is stalling", AlertType.kWarning);

  private Angle m_setpoint = Degrees.zero();
  private double m_simAngleRadians = kMinimumAngle.in(Radians);

  private static final double kSimMaxVelocityRadiansPerSecond = Math.toRadians(45.0);

  public Hood(int motorId, int encoderId) {
    m_motor = new TalonFX(motorId);
    m_encoder = new CANcoder(encoderId);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0 = new Slot0Configs()
        .withKP(800)
        .withKI(0)
        .withKD(18)
        .withKS(0.395)
        .withKV(30)
        .withKG(0.015)
        .withGravityArmPositionOffset(Degrees.of(11))
        .withGravityType(GravityTypeValue.Arm_Cosine);

    config.Feedback = new FeedbackConfigs()
        .withSensorToMechanismRatio(1.0)
        .withRotorToSensorRatio(kRotorToHoodRatio)
        .withFusedCANcoder(m_encoder);

    config.MotorOutput = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);

    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(40));

    config.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(RPM.of(600))
        .withMotionMagicAcceleration(RPM.per(Second).of(700));

    config.SoftwareLimitSwitch = createSoftLimitConfig(true);
    config.OpenLoopRamps = new OpenLoopRampsConfigs()
        .withVoltageOpenLoopRampPeriod(Seconds.of(0.25));
    config.ClosedLoopRamps = new ClosedLoopRampsConfigs()
        .withVoltageClosedLoopRampPeriod(Seconds.of(0.25));

    m_motor.getConfigurator().apply(config);

    m_currentSpikeTrigger = new Trigger(this::isCurrentSpiking);
    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(Degrees.of(1)));
    setAngle(kMinimumAngle);
  }

  private static SoftwareLimitSwitchConfigs createSoftLimitConfig(boolean lowerLimitEnabled) {
    return new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(kMaximumAngle)
        .withReverseSoftLimitEnable(lowerLimitEnabled)
        .withReverseSoftLimitThreshold(kMinimumAngle);
  }

  private boolean isCurrentSpiking() {
    double filteredCurrent = m_currentSpikeFilter.calculate(m_motor.getStatorCurrent().getValue().in(Amps));
    return filteredCurrent > 20.0;
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> setPID(values[0], values[1], values[2], values[3], values[4], values[5]),
        m_kP, m_kI, m_kD, m_kS, m_kV, m_kG);

    if (RobotBase.isSimulation()) {
      double error = m_setpoint.in(Radians) - m_simAngleRadians;
      double maximumStep = kSimMaxVelocityRadiansPerSecond * 0.02;
      m_simAngleRadians += Math.max(-maximumStep, Math.min(maximumStep, error));
    }

    var encoderAbsolutePosition = m_encoder.getAbsolutePosition();
    encoderAbsolutePosition.refresh();

    m_motorConnectionAlert.set(!m_motor.isAlive());
    m_encoderConnectionAlert.set(!encoderAbsolutePosition.getStatus().isOK());
    m_stallAlert.set(m_motor.getStatorCurrent().getValue().gt(Amps.of(30)));

    Logger.recordOutput("Hood/CurrentSpike", isCurrentSpiking());
    Logger.recordOutput("Hood/IsNearSetpoint", m_isNearSetpointTrigger.getAsBoolean());
  }

  @AutoLogOutput(key = "Hood/Position")
  public Angle getAngle() {
    if (RobotBase.isSimulation()) {
      return Radians.of(m_simAngleRadians);
    }
    return m_motor.getPosition().getValue();
  }

  @AutoLogOutput(key = "Hood/AbsolutePosition")
  public Angle getAbsoluteAngle() {
    if (RobotBase.isSimulation()) {
      return Radians.of(m_simAngleRadians);
    }
    return m_encoder.getAbsolutePosition().getValue();
  }

  /** Returns hood travel relative to the physical minimum for 3D visualization. */
  public Angle getAngleFromMinimum() {
    return getAngle().minus(kMinimumAngle);
  }

  @AutoLogOutput(key = "Hood/Velocity")
  public AngularVelocity getVelocity() {
    return m_motor.getVelocity().getValue();
  }

  @AutoLogOutput(key = "Hood/AppliedVoltage")
  public Voltage getAppliedVoltage() {
    return m_motor.getMotorVoltage().getValue();
  }

  @AutoLogOutput(key = "Hood/StatorCurrent")
  public Current getStatorCurrent() {
    return m_motor.getStatorCurrent().getValue();
  }

  @AutoLogOutput(key = "Hood/SupplyCurrent")
  public Current getSupplyCurrent() {
    return m_motor.getSupplyCurrent().getValue();
  }

  @AutoLogOutput(key = "Hood/Setpoint")
  public Angle getSetpointAngle() {
    return m_setpoint;
  }

  public boolean isNearSetpoint(Angle tolerance) {
    return getAngle().isNear(m_setpoint, tolerance);
  }

  public Trigger isNearSetpointTrigger() {
    return m_isNearSetpointTrigger;
  }

  public Trigger getCurrentSpikeTrigger() {
    return m_currentSpikeTrigger;
  }

  public Trigger isNearTrigger(Supplier<Angle> angle, Angle tolerance) {
    return new Trigger(() -> getAngle().isNear(angle.get(), tolerance));
  }

  public void setAngle(Angle angle) {
    m_setpoint = angle;
    m_motor.setControl(m_positionRequest.withPosition(angle.in(Rotations)));
  }

  public Command setAngleCommand(Angle angle) {
    return runOnce(() -> setAngle(angle));
  }

  public Command setMinimumAngleCommand() {
    return setAngleCommand(kMinimumAngle);
  }

  public Command setMaximumAngleCommand() {
    return setAngleCommand(kMaximumAngle);
  }

  public Command setAngleCommand(Supplier<Angle> angleSupplier) {
    return run(() -> setAngle(angleSupplier.get()));
  }

  public void setVoltage(Voltage voltage) {
    m_motor.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> setVoltage(voltage));
  }

  public void setEncoderPosition(Angle angle) {
    if (RobotBase.isSimulation()) {
      m_simAngleRadians = angle.in(Radians);
    }
    m_motor.setPosition(angle);
    m_encoder.setPosition(angle);
  }

  public Command zeroEncoderCommand() {
    return runOnce(() -> setEncoderPosition(Degrees.zero()));
  }

  public void setLowerSoftLimitEnabled(boolean enabled) {
    m_motor.getConfigurator().apply(createSoftLimitConfig(enabled));
  }

  public Command setLowerSoftLimitEnabledCommand(boolean enabled) {
    return runOnce(() -> setLowerSoftLimitEnabled(enabled));
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV, double kG) {
    Slot0Configs gains = new Slot0Configs()
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV)
        .withKG(kG)
        .withGravityArmPositionOffset(Degrees.of(11))
        .withGravityType(GravityTypeValue.Arm_Cosine);
    m_motor.getConfigurator().apply(gains);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
