package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX motor;
  private final CANcoder encoder;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final StatusSignal<Angle> position;
  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private Angle setpoint = Degrees.of(90);

  public IntakePivotIOTalonFX(int motorId, int encoderId) {
    motor = new TalonFX(motorId);
    encoder = new CANcoder(encoderId);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 0.0237;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Degrees.of(90).in(Rotations);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    absolutePosition = encoder.getAbsolutePosition();
    velocity = encoder.getVelocity();
    voltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, absolutePosition, velocity, voltage, statorCurrent, supplyCurrent);
    motor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, absolutePosition, velocity, voltage, statorCurrent, supplyCurrent);
    inputs.position = absolutePosition.getValue();
    inputs.absolutePosition = absolutePosition.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.setpoint = setpoint;
    inputs.motorConnected = motor.isAlive();
    inputs.encoderConnected = absolutePosition.getStatus().isOK();
  }

  @Override
  public void setPosition(Angle angle) {
    setpoint = angle;
    motor.setControl(positionRequest.withPosition(angle.in(Rotations)));
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kG) {
    motor
        .getConfigurator()
        .apply(new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKG(kG));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
