package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.001, 2), motor);
  private final PIDController pid = new PIDController(0.1, 0, 0);
  private AngularVelocity setpoint = RPM.zero();
  private double volts;
  private boolean closedLoop;

  @Override
  public void updateInputs(FeederIOInputs i) {
    if (closedLoop)
      volts = MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM(), setpoint.in(RPM)), -12, 12);
    sim.setInputVoltage(volts);
    sim.update(0.02);
    i.leaderVelocity = RPM.of(sim.getAngularVelocityRPM());
    i.followerVelocity = i.leaderVelocity;
    i.leaderAppliedVoltage = Volts.of(volts);
    i.followerAppliedVoltage = i.leaderAppliedVoltage;
    i.leaderStatorCurrent = Amps.of(sim.getCurrentDrawAmps());
    i.followerStatorCurrent = i.leaderStatorCurrent;
    i.leaderSupplyCurrent = i.leaderStatorCurrent;
    i.followerSupplyCurrent = i.leaderStatorCurrent;
    i.setpoint = setpoint;
    i.leaderConnected = true;
    i.followerConnected = true;
  }

  @Override
  public void setVelocity(AngularVelocity v) {
    setpoint = v;
    closedLoop = true;
  }

  @Override
  public void setVoltage(Voltage v) {
    setpoint = RPM.zero();
    closedLoop = false;
    volts = MathUtil.clamp(v.in(Volts), -12, 12);
  }

  @Override
  public void setPID(double p, double i, double d) {
    pid.setPID(p, i, d);
  }

  @Override
  public void stop() {
    setpoint = RPM.zero();
    closedLoop = false;
    volts = 0;
  }
}
