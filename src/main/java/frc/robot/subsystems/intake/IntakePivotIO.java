package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  @AutoLog
  class IntakePivotIOInputs {
    public Angle position = Degrees.zero();
    public Angle absolutePosition = Degrees.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
    public Angle setpoint = Degrees.zero();
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
  }

  default void updateInputs(IntakePivotIOInputs inputs) {}

  default void setPosition(Angle angle) {}

  default void setPID(double kP, double kI, double kD, double kS, double kG) {}

  default void stop() {}
}
