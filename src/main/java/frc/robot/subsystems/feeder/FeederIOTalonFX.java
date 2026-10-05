package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX leader, follower;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final StatusSignal<AngularVelocity> leaderVelocity, followerVelocity;
  private final StatusSignal<Voltage> leaderVoltage, followerVoltage;
  private final StatusSignal<Current> leaderStator, followerStator, leaderSupply, followerSupply;
  private AngularVelocity setpoint = RPM.zero();

  public FeederIOTalonFX(int leaderId, int followerId) {
    leader = new TalonFX(leaderId);
    follower = new TalonFX(followerId);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 2.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);
    follower.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
    leaderVelocity = leader.getVelocity();
    followerVelocity = follower.getVelocity();
    leaderVoltage = leader.getMotorVoltage();
    followerVoltage = follower.getMotorVoltage();
    leaderStator = leader.getStatorCurrent();
    followerStator = follower.getStatorCurrent();
    leaderSupply = leader.getSupplyCurrent();
    followerSupply = follower.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leaderVelocity,
        followerVelocity,
        leaderVoltage,
        followerVoltage,
        leaderStator,
        followerStator,
        leaderSupply,
        followerSupply);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputs i) {
    BaseStatusSignal.refreshAll(
        leaderVelocity,
        followerVelocity,
        leaderVoltage,
        followerVoltage,
        leaderStator,
        followerStator,
        leaderSupply,
        followerSupply);
    i.leaderVelocity = leaderVelocity.getValue();
    i.followerVelocity = followerVelocity.getValue();
    i.leaderAppliedVoltage = leaderVoltage.getValue();
    i.followerAppliedVoltage = followerVoltage.getValue();
    i.leaderStatorCurrent = leaderStator.getValue();
    i.followerStatorCurrent = followerStator.getValue();
    i.leaderSupplyCurrent = leaderSupply.getValue();
    i.followerSupplyCurrent = followerSupply.getValue();
    i.setpoint = setpoint;
    i.leaderConnected = leader.isAlive();
    i.followerConnected = follower.isAlive();
  }

  @Override
  public void setVelocity(AngularVelocity v) {
    setpoint = v;
    leader.setControl(velocityRequest.withVelocity(v.in(RotationsPerSecond)));
  }

  @Override
  public void setVoltage(Voltage v) {
    setpoint = RPM.zero();
    leader.setControl(voltageRequest.withOutput(v.in(Volts)));
  }

  @Override
  public void setPID(double p, double i, double d) {
    leader.getConfigurator().apply(new Slot0Configs().withKP(p).withKI(i).withKD(d));
  }

  @Override
  public void stop() {
    setpoint = RPM.zero();
    leader.stopMotor();
  }
}
