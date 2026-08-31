package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotor motor = DCMotor.getKrakenX60Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.001, 1.0), motor);
  private Voltage setpoint = Volts.zero();

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    sim.setInputVoltage(setpoint.in(Volts));
    sim.update(0.02);
    inputs.velocity = RPM.of(sim.getAngularVelocityRPM());
    inputs.appliedVoltage = setpoint;
    inputs.statorCurrent = Amps.of(sim.getCurrentDrawAmps());
    inputs.supplyCurrent = inputs.statorCurrent;
    inputs.setpoint = setpoint;
    inputs.motorConnected = true;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    setpoint = Volts.of(Math.max(-12, Math.min(12, voltage.in(Volts))));
  }

  @Override
  public void stop() {
    setpoint = Volts.zero();
  }
}
