package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  class FeederIOInputs {
    public AngularVelocity leaderVelocity = RPM.zero();
    public AngularVelocity followerVelocity = RPM.zero();
    public Voltage leaderAppliedVoltage = Volts.zero();
    public Voltage followerAppliedVoltage = Volts.zero();
    public Current leaderStatorCurrent = Amps.zero();
    public Current followerStatorCurrent = Amps.zero();
    public Current leaderSupplyCurrent = Amps.zero();
    public Current followerSupplyCurrent = Amps.zero();
    public AngularVelocity setpoint = RPM.zero();
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
  }

  default void updateInputs(FeederIOInputs inputs) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void setVoltage(Voltage voltage) {}

  default void setPID(double kP, double kI, double kD) {}

  default void stop() {}
}
