package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX motor;
  private final VoltageOut request = new VoltageOut(0).withEnableFOC(true);
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private Voltage setpoint = Volts.zero();

  public IntakeRollerIOTalonFX(int motorId) {
    motor = new TalonFX(motorId);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(1.0);
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(90));
    config.OpenLoopRamps = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(Seconds.of(2));
    motor.getConfigurator().apply(config);
    velocity = motor.getVelocity();
    appliedVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, velocity, appliedVoltage, statorCurrent, supplyCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, appliedVoltage, statorCurrent, supplyCurrent);
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = appliedVoltage.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.setpoint = setpoint;
    inputs.motorConnected = motor.isAlive();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    setpoint = voltage;
    motor.setControl(request.withOutput(voltage.in(Volts)));
  }

  @Override
  public void stop() {
    setpoint = Volts.zero();
    motor.stopMotor();
  }
}
