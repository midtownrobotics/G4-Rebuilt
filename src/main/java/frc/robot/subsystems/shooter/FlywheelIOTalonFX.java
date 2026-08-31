package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX hood, back;
  private final VelocityVoltage hoodVel = new VelocityVoltage(0).withEnableFOC(true),
      backVel = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut hoodVolts = new VoltageOut(0), backVolts = new VoltageOut(0);
  private final StatusSignal<AngularVelocity> hv, bv;
  private final StatusSignal<Voltage> hvolt, bvolt;
  private final StatusSignal<Current> hs, bs, hsp, bsp;
  private AngularVelocity setpoint = RPM.zero();

  public FlywheelIOTalonFX(int hoodId, int backId) {
    hood = new TalonFX(hoodId);
    back = new TalonFX(backId);
    configure(hood, 28.0 / 30.0);
    configure(back, 1);
    hv = hood.getVelocity();
    bv = back.getVelocity();
    hvolt = hood.getMotorVoltage();
    bvolt = back.getMotorVoltage();
    hs = hood.getStatorCurrent();
    bs = back.getStatorCurrent();
    hsp = hood.getSupplyCurrent();
    bsp = back.getSupplyCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50, hv, bv, hvolt, bvolt, hs, bs, hsp, bsp);
    hood.optimizeBusUtilization();
    back.optimizeBusUtilization();
  }

  private static void configure(TalonFX m, double ratio) {
    TalonFXConfiguration c = new TalonFXConfiguration();
    c.Feedback.SensorToMechanismRatio = ratio;
    c.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m.getConfigurator().apply(c);
  }

  @Override
  public void updateInputs(FlywheelIOInputs x) {
    BaseStatusSignal.refreshAll(hv, bv, hvolt, bvolt, hs, bs, hsp, bsp);
    x.hoodVelocity = hv.getValue();
    x.backVelocity = bv.getValue();
    x.hoodAppliedVoltage = hvolt.getValue();
    x.backAppliedVoltage = bvolt.getValue();
    x.hoodStatorCurrent = hs.getValue();
    x.backStatorCurrent = bs.getValue();
    x.hoodSupplyCurrent = hsp.getValue();
    x.backSupplyCurrent = bsp.getValue();
    x.setpoint = setpoint;
    x.hoodConnected = hood.isAlive();
    x.backConnected = back.isAlive();
  }

  @Override
  public void setVelocity(AngularVelocity v) {
    setpoint = v;
    double r = v.in(RotationsPerSecond);
    hood.setControl(hoodVel.withVelocity(r));
    back.setControl(backVel.withVelocity(r));
  }

  @Override
  public void setVoltage(Voltage v) {
    setpoint = RPM.zero();
    hood.setControl(hoodVolts.withOutput(v.in(Volts)));
    back.setControl(backVolts.withOutput(v.in(Volts)));
  }

  @Override
  public void setPID(double p, double i, double d, double s, double v) {
    Slot0Configs g = new Slot0Configs().withKP(p).withKI(i).withKD(d).withKS(s).withKV(v);
    hood.getConfigurator().apply(g);
    back.getConfigurator().apply(g);
  }

  @Override
  public void stop() {
    setpoint = RPM.zero();
    hood.stopMotor();
    back.stopMotor();
  }
}
