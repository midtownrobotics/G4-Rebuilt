package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakePivot extends SubsystemBase{
  private final TalonFX m_motor;
  private final CANcoder m_encoder;
  private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);
  private Angle m_setpoint = State.START.angle();
  private double m_simAngleRadians = State.START.angle().in(Radians);

  private static final double kSimMaxVelocityRadiansPerSecond = Math.toRadians(180.0);

  @AutoLogOutput (key = "IntakePivot/State")
  public State state = State.START;

  public enum State {
    START(Degrees.of(90)),
    INTAKING(Degrees.of(0)),
    NOT_INTAKING(Degrees.of(90));

    private final Angle m_angle;

    State(Angle angle) {
      m_angle = angle;
    }

    public Angle angle() {
      return m_angle;
    }
  }

  public IntakePivot(int motorId, int encoderId) {
    m_motor = new TalonFX(motorId);
    m_encoder = new CANcoder(encoderId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = 0.0237;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.StatorCurrentLimit = Amps.of(120).in(Amps);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Amps.of(70).in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Degrees.of(90).in(Rotations);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    m_motor.getConfigurator().apply(config);

    setState(State.START);
    Logger.recordOutput("IntakePivot/AbsolutePosition", this.getAngle());
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      double error = m_setpoint.in(Radians) - m_simAngleRadians;
      double maximumStep = kSimMaxVelocityRadiansPerSecond * 0.02;
      m_simAngleRadians += Math.max(-maximumStep, Math.min(maximumStep, error));
    }
  }

  public void setAngle(Angle angle) {
    m_setpoint = angle;
    m_motor.setControl(m_positionRequest.withPosition(angle.in(Rotations)));
  }

  public void setState(State state) {
    this.state = state;
    setAngle(state.angle());
  }

  public Command setStateCommand(State state) {
    return Commands.runOnce(() -> setState(state), this);
  }

  public Command setAngleCommand(Angle angle) {
    return Commands.runOnce(() -> setAngle(angle), this);
  }

  @AutoLogOutput (key = "IntakePivot/Angle")
  public Angle getAngle() {
    if (RobotBase.isSimulation()) {
      return Radians.of(m_simAngleRadians);
    }
    return m_encoder.getAbsolutePosition().getValue();
  }

  @AutoLogOutput (key = "IntakePivot/Velocity")
  public AngularVelocity getVelocity() {
    return m_encoder.getVelocity().getValue();
  }

  @AutoLogOutput (key = "IntakePivot/SupplyVoltage")
  public Voltage getVoltage() {
    return m_encoder.getSupplyVoltage().getValue();
  }

  @AutoLogOutput(key = "IntakePivot/AppliedVoltage")
  public Voltage getAppliedVoltage() {
    return m_motor.getMotorVoltage().getValue();
  }

  @AutoLogOutput(key = "IntakePivot/StatorCurrent")
  public Current getStatorCurrent() {
    return m_motor.getStatorCurrent().getValue();
  }

  @AutoLogOutput(key = "IntakePivot/SupplyCurrent")
  public Current getSupplyCurrent() {
    return m_motor.getSupplyCurrent().getValue();
  }

  @AutoLogOutput(key = "IntakePivot/Setpoint")
  public Angle getSetpoint() {
    return m_setpoint;
  }

  public void start() {
    setState(State.START);
  }
}
