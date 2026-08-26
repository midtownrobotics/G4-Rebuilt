package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
  private static final double kGearRatio = 1.0;

  private final TalonFX m_motor;
  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> m_velocitySignal;
  private final StatusSignal<Voltage> m_appliedVoltageSignal;
  private final StatusSignal<Current> m_statorCurrentSignal;
  private final StatusSignal<Current> m_supplyCurrentSignal;

  private final Alert m_connectionAlert =
      new Alert("Intake roller motor is not connected", AlertType.kWarning);

  private Voltage m_setpoint = Volts.zero();

  public IntakeRoller(int motorId) {
    this(motorId, new CANBus("rio"));
  }

  public IntakeRoller(int motorId, CANBus canBus) {
    m_motor = new TalonFX(motorId, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(kGearRatio);
    config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(90));
    config.OpenLoopRamps =
        new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(Seconds.of(2.0));
    m_motor.getConfigurator().apply(config);

    m_velocitySignal = m_motor.getVelocity();
    m_appliedVoltageSignal = m_motor.getMotorVoltage();
    m_statorCurrentSignal = m_motor.getStatorCurrent();
    m_supplyCurrentSignal = m_motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_velocitySignal,
        m_appliedVoltageSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal);
    m_motor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
        m_velocitySignal,
        m_appliedVoltageSignal,
        m_statorCurrentSignal,
        m_supplyCurrentSignal);
    m_connectionAlert.set(!m_motor.isAlive());
  }

  @AutoLogOutput(key = "IntakeRoller/Velocity")
  public AngularVelocity getVelocity() {
    return m_velocitySignal.getValue();
  }

  @AutoLogOutput(key = "IntakeRoller/AppliedVoltage")
  public Voltage getAppliedVoltage() {
    return m_appliedVoltageSignal.getValue();
  }

  @AutoLogOutput(key = "IntakeRoller/StatorCurrent")
  public Current getStatorCurrent() {
    return m_statorCurrentSignal.getValue();
  }

  @AutoLogOutput(key = "IntakeRoller/SupplyCurrent")
  public Current getSupplyCurrent() {
    return m_supplyCurrentSignal.getValue();
  }

  @AutoLogOutput(key = "IntakeRoller/Setpoint")
  public Voltage getSetpoint() {
    return m_setpoint;
  }

  public void setVoltage(Voltage voltage) {
    m_setpoint = voltage;
    m_motor.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> setVoltage(voltage));
  }

  public void stop() {
    m_setpoint = Volts.zero();
    m_motor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
