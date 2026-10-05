package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public Angle position = Degrees.zero(),
        absolutePosition = Degrees.zero(),
        setpoint = Degrees.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero(), supplyCurrent = Amps.zero();
    public boolean motorConnected = false, encoderConnected = false;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setVoltage(Voltage voltage) {}

  default void setEncoderPosition(Angle angle) {}

  default void setLowerSoftLimitEnabled(boolean enabled) {}

  default void setPID(double p, double i, double d, double s, double v, double g) {}

  default void stop() {}
}
