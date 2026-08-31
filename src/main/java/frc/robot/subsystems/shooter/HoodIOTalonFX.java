package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;

public class HoodIOTalonFX implements HoodIO {
  private static final double RATIO = 62.0 / 14.0;
  private final TalonFX motor;
  private final CANcoder encoder;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final StatusSignal<Angle> position, absolute;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> stator, supply;
  private Angle setpoint = Hood.kMinimumAngle;

  public HoodIOTalonFX(int motorId, int encoderId) {
    motor = new TalonFX(motorId);
    encoder = new CANcoder(encoderId);
    TalonFXConfiguration c = new TalonFXConfiguration();
    c.Slot0 =
        new Slot0Configs()
            .withKP(800)
            .withKD(18)
            .withKS(.395)
            .withKV(30)
            .withKG(.015)
            .withGravityArmPositionOffset(Degrees.of(11))
            .withGravityType(GravityTypeValue.Arm_Cosine);
    c.Feedback =
        new FeedbackConfigs()
            .withSensorToMechanismRatio(1)
            .withRotorToSensorRatio(RATIO)
            .withFusedCANcoder(encoder);
    c.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    c.CurrentLimits.StatorCurrentLimitEnable = true;
    c.CurrentLimits.StatorCurrentLimit = 40;
    c.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(RPM.of(600))
            .withMotionMagicAcceleration(RPM.per(Second).of(700));
    c.SoftwareLimitSwitch = limits(true);
    c.OpenLoopRamps.VoltageOpenLoopRampPeriod = .25;
    c.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .25;
    motor.getConfigurator().apply(c);
    position = motor.getPosition();
    absolute = encoder.getAbsolutePosition();
    velocity = motor.getVelocity();
    voltage = motor.getMotorVoltage();
    stator = motor.getStatorCurrent();
    supply = motor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, position, absolute, velocity, voltage, stator, supply);
    motor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  private static SoftwareLimitSwitchConfigs limits(boolean lower) {
    return new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(Hood.kMaximumAngle)
        .withReverseSoftLimitEnable(lower)
        .withReverseSoftLimitThreshold(Hood.kMinimumAngle);
  }

  @Override
  public void updateInputs(HoodIOInputs x) {
    BaseStatusSignal.refreshAll(position, absolute, velocity, voltage, stator, supply);
    x.position = position.getValue();
    x.absolutePosition = absolute.getValue();
    x.velocity = velocity.getValue();
    x.appliedVoltage = voltage.getValue();
    x.statorCurrent = stator.getValue();
    x.supplyCurrent = supply.getValue();
    x.setpoint = setpoint;
    x.motorConnected = motor.isAlive();
    x.encoderConnected = absolute.getStatus().isOK();
  }

  @Override
  public void setPosition(Angle a) {
    setpoint = a;
    motor.setControl(positionRequest.withPosition(a.in(Rotations)));
  }

  @Override
  public void setVoltage(Voltage v) {
    motor.setControl(voltageRequest.withOutput(v.in(Volts)));
  }

  @Override
  public void setEncoderPosition(Angle a) {
    motor.setPosition(a);
    encoder.setPosition(a);
  }

  @Override
  public void setLowerSoftLimitEnabled(boolean e) {
    motor.getConfigurator().apply(limits(e));
  }

  @Override
  public void setPID(double p, double i, double d, double s, double v, double g) {
    motor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(p)
                .withKI(i)
                .withKD(d)
                .withKS(s)
                .withKV(v)
                .withKG(g)
                .withGravityArmPositionOffset(Degrees.of(11))
                .withGravityType(GravityTypeValue.Arm_Cosine));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
