package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  class IntakeRollerIOInputs {
    public AngularVelocity velocity = RPM.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
    public Voltage setpoint = Volts.zero();
    public boolean motorConnected = false;
  }

  default void updateInputs(IntakeRollerIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}

  default void stop() {}
}
