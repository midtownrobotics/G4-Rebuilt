package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.security.Key;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;

public class IntakePivot {
  private final TalonFX m_motor;
  private final CANcoder m_encoder;
  private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

  @AutoLogOutput (key = "IntakePivot/State")
  private State state = State.START;

  public enum State {
    START(Degrees.of(115)),
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
  }

  public void setAngle(Angle angle) {
    m_motor.setControl(m_positionRequest.withPosition(angle.in(Rotations)));
  }

  @AutoLogOutput (key = "IntakePivot/Angle")
  public Angle getAngle() {
    return m_encoder.getAbsolutePosition().getValue();
  }

  @AutoLogOutput (key = "IntakePivot/Voltage")
  public Voltage getVoltage() {
    return m_encoder.getSupplyVoltage();
  }

  @AutoLogOutput (key = "IntakePivot/AngularVelocity")
  public AngularVelocity getVelocity() {
    return m_encoder.getVelocity();
  }

  public void start() {
    setAngle(State.START.angle());
  }
}
