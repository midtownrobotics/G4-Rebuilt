package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, 1), motor);
  private final PIDController pid = new PIDController(0, 0, 0);
  private AngularVelocity setpoint = RPM.zero();
  private double volts;
  private boolean closed;

  @Override
  public void updateInputs(FlywheelIOInputs x) {
    if (closed)
      volts = MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM(), setpoint.in(RPM)), -12, 12);
    sim.setInputVoltage(volts);
    sim.update(.02);
    x.hoodVelocity = RPM.of(sim.getAngularVelocityRPM());
    x.backVelocity = x.hoodVelocity;
    x.hoodAppliedVoltage = Volts.of(volts);
    x.backAppliedVoltage = x.hoodAppliedVoltage;
    x.hoodStatorCurrent = Amps.of(sim.getCurrentDrawAmps());
    x.backStatorCurrent = x.hoodStatorCurrent;
    x.hoodSupplyCurrent = x.hoodStatorCurrent;
    x.backSupplyCurrent = x.hoodStatorCurrent;
    x.setpoint = setpoint;
    x.hoodConnected = true;
    x.backConnected = true;
  }

  @Override
  public void setVelocity(AngularVelocity v) {
    setpoint = v;
    closed = true;
  }

  @Override
  public void setVoltage(Voltage v) {
    setpoint = RPM.zero();
    closed = false;
    volts = MathUtil.clamp(v.in(Volts), -12, 12);
  }

  @Override
  public void setPID(double p, double i, double d, double s, double v) {
    pid.setPID(p, i, d);
  }

  @Override
  public void stop() {
    closed = false;
    volts = 0;
    setpoint = RPM.zero();
  }
}
