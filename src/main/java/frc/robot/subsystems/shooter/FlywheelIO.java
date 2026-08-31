package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelIOInputs {
    public AngularVelocity hoodVelocity = RPM.zero(),
        backVelocity = RPM.zero(),
        setpoint = RPM.zero();
    public Voltage hoodAppliedVoltage = Volts.zero(), backAppliedVoltage = Volts.zero();
    public Current hoodStatorCurrent = Amps.zero(), backStatorCurrent = Amps.zero();
    public Current hoodSupplyCurrent = Amps.zero(), backSupplyCurrent = Amps.zero();
    public boolean hoodConnected = false, backConnected = false;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void setVoltage(Voltage voltage) {}

  default void setPID(double p, double i, double d, double s, double v) {}

  default void stop() {}
  
}
