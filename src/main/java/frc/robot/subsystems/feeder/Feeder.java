package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;

public class Feeder extends SubsystemBase {
  private static final double kGearRatio = 2.0;
  private static final double kSimMomentOfInertia = 0.001;
  private static final Voltage kFeedVoltage = Volts.of(10.0);
  private static final Voltage kReverseVoltage = Volts.of(-10.0);

  private final LoggedTunableNumber m_kP = new LoggedTunableNumber("Feeder/kP", 0.1);
  private final LoggedTunableNumber m_kI = new LoggedTunableNumber("Feeder/kI", 0.0);
  private final LoggedTunableNumber m_kD = new LoggedTunableNumber("Feeder/kD", 0.0);

  private final TalonFX m_leaderMotor;
  private final TalonFX m_followerMotor;

  private final VelocityVoltage m_velocityRequest =
      new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> m_leaderVelocitySignal;
  private final StatusSignal<AngularVelocity> m_followerVelocitySignal;
  private final StatusSignal<Voltage> m_leaderVoltageSignal;
  private final StatusSignal<Voltage> m_followerVoltageSignal;
  private final StatusSignal<Current> m_leaderStatorCurrentSignal;
  private final StatusSignal<Current> m_followerStatorCurrentSignal;
  private final StatusSignal<Current> m_leaderSupplyCurrentSignal;
  private final StatusSignal<Current> m_followerSupplyCurrentSignal;

  private final Alert m_leaderConnectionAlert =
      new Alert("Feeder leader motor is not connected", AlertType.kWarning);
  private final Alert m_followerConnectionAlert =
      new Alert("Feeder follower motor is not connected", AlertType.kWarning);
  private final Alert m_stallAlert =
      new Alert("Feeder is stalling", AlertType.kWarning);

  private final DCMotorSim m_sim;
  private final PIDController m_simController = new PIDController(0.1, 0.0, 0.0);
  private double m_simAppliedVolts = 0.0;
  private boolean m_simClosedLoop = false;

  private AngularVelocity m_setpoint = RPM.zero();

  public Feeder(int leaderMotorId, int followerMotorId) {
    this(leaderMotorId, followerMotorId, new CANBus("rio"));
  }

  public Feeder(int leaderMotorId, int followerMotorId, CANBus canBus) {
    m_leaderMotor = new TalonFX(leaderMotorId, canBus);
    m_followerMotor = new TalonFX(followerMotorId, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(kGearRatio);
    config.MotorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(120))
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(40));

    m_leaderMotor.getConfigurator().apply(config);
    m_followerMotor.getConfigurator().apply(config);

    m_leaderVelocitySignal = m_leaderMotor.getVelocity();
    m_followerVelocitySignal = m_followerMotor.getVelocity();
    m_leaderVoltageSignal = m_leaderMotor.getMotorVoltage();
    m_followerVoltageSignal = m_followerMotor.getMotorVoltage();
    m_leaderStatorCurrentSignal = m_leaderMotor.getStatorCurrent();
    m_followerStatorCurrentSignal = m_followerMotor.getStatorCurrent();
    m_leaderSupplyCurrentSignal = m_leaderMotor.getSupplyCurrent();
    m_followerSupplyCurrentSignal = m_followerMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        m_leaderVelocitySignal,
        m_followerVelocitySignal,
        m_leaderVoltageSignal,
        m_followerVoltageSignal,
        m_leaderStatorCurrentSignal,
        m_followerStatorCurrentSignal,
        m_leaderSupplyCurrentSignal,
        m_followerSupplyCurrentSignal);

    m_followerMotor.setControl(
        new Follower(m_leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    m_leaderMotor.optimizeBusUtilization();
    m_followerMotor.optimizeBusUtilization();

    DCMotor simMotor = DCMotor.getKrakenX60(1);
    m_sim = RobotBase.isSimulation()
        ? new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                simMotor, kSimMomentOfInertia, kGearRatio),
            simMotor)
        : null;
  }

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(), values -> setPID(values[0], values[1], values[2]), m_kP, m_kI, m_kD);

    BaseStatusSignal.refreshAll(
        m_leaderVelocitySignal,
        m_followerVelocitySignal,
        m_leaderVoltageSignal,
        m_followerVoltageSignal,
        m_leaderStatorCurrentSignal,
        m_followerStatorCurrentSignal,
        m_leaderSupplyCurrentSignal,
        m_followerSupplyCurrentSignal);

    boolean highCurrent = getLeaderStatorCurrent().gt(Amps.of(30));
    boolean notMoving = getLeaderVelocity().abs(RPM) < 120.0;
    m_leaderConnectionAlert.set(!m_leaderMotor.isAlive());
    m_followerConnectionAlert.set(!m_followerMotor.isAlive());
    m_stallAlert.set(highCurrent && notMoving);
  }

  @Override
  public void simulationPeriodic() {
    if (m_simClosedLoop) {
      m_simAppliedVolts = MathUtil.clamp(
          m_simController.calculate(
              m_sim.getAngularVelocityRPM(), m_setpoint.in(RPM)),
          -12.0,
          12.0);
    }

    m_sim.setInputVoltage(m_simAppliedVolts);
    m_sim.update(0.02);
  }

  @AutoLogOutput(key = "Feeder/LeaderVelocity")
  public AngularVelocity getLeaderVelocity() {
    return m_sim != null
        ? RPM.of(m_sim.getAngularVelocityRPM())
        : m_leaderVelocitySignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/FollowerVelocity")
  public AngularVelocity getFollowerVelocity() {
    return m_sim != null
        ? RPM.of(m_sim.getAngularVelocityRPM())
        : m_followerVelocitySignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/LeaderAppliedVoltage")
  public Voltage getLeaderAppliedVoltage() {
    return m_sim != null ? Volts.of(m_simAppliedVolts) : m_leaderVoltageSignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/FollowerAppliedVoltage")
  public Voltage getFollowerAppliedVoltage() {
    return m_sim != null ? Volts.of(m_simAppliedVolts) : m_followerVoltageSignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/LeaderStatorCurrent")
  public Current getLeaderStatorCurrent() {
    return m_sim != null
        ? Amps.of(m_sim.getCurrentDrawAmps())
        : m_leaderStatorCurrentSignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/FollowerStatorCurrent")
  public Current getFollowerStatorCurrent() {
    return m_sim != null
        ? Amps.of(m_sim.getCurrentDrawAmps())
        : m_followerStatorCurrentSignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/LeaderSupplyCurrent")
  public Current getLeaderSupplyCurrent() {
    return m_sim != null
        ? Amps.of(m_sim.getCurrentDrawAmps())
        : m_leaderSupplyCurrentSignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/FollowerSupplyCurrent")
  public Current getFollowerSupplyCurrent() {
    return m_sim != null
        ? Amps.of(m_sim.getCurrentDrawAmps())
        : m_followerSupplyCurrentSignal.getValue();
  }

  @AutoLogOutput(key = "Feeder/Setpoint")
  public AngularVelocity getSetpoint() {
    return m_setpoint;
  }

  public void setVelocity(AngularVelocity velocity) {
    m_setpoint = velocity;
    if (m_sim != null) {
      m_simClosedLoop = true;
      return;
    }
    m_leaderMotor.setControl(
        m_velocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return run(() -> setVelocity(velocity)).finallyDo(this::stop);
  }

  public void setVoltage(Voltage voltage) {
    m_setpoint = RPM.zero();
    if (m_sim != null) {
      m_simClosedLoop = false;
      m_simAppliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
      return;
    }
    m_leaderMotor.setControl(m_voltageRequest.withOutput(voltage.in(Volts)));
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> setVoltage(voltage)).finallyDo(this::stop);
  }

  public Command runForwardCommand() {
    return setVoltageCommand(kFeedVoltage);
  }

  public Command runReverseCommand() {
    return setVoltageCommand(kReverseVoltage);
  }

  public void setPID(double kP, double kI, double kD) {
    m_simController.setPID(kP, kI, kD);
    m_leaderMotor.getConfigurator().apply(
        new Slot0Configs().withKP(kP).withKI(kI).withKD(kD));
  }

  public void stop() {
    m_setpoint = RPM.zero();
    if (m_sim != null) {
      m_simClosedLoop = false;
      m_simAppliedVolts = 0.0;
      return;
    }
    m_leaderMotor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
