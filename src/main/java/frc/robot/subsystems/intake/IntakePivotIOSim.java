package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

public class IntakePivotIOSim implements IntakePivotIO {
  private static final double MAX_VELOCITY_RAD_PER_SEC = Math.toRadians(180.0);
  private Angle position = Degrees.of(90);
  private Angle setpoint = Degrees.of(90);
  private double previousRadians = position.in(Radians);

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    double current = position.in(Radians);
    double error = setpoint.in(Radians) - current;
    double step =
        Math.max(
            -MAX_VELOCITY_RAD_PER_SEC * 0.02, Math.min(MAX_VELOCITY_RAD_PER_SEC * 0.02, error));
    position = Radians.of(current + step);
    inputs.position = position;
    inputs.absolutePosition = position;
    inputs.velocity =
        DegreesPerSecond.of(Math.toDegrees((position.in(Radians) - previousRadians) / 0.02));
    inputs.setpoint = setpoint;
    inputs.motorConnected = true;
    inputs.encoderConnected = true;
    previousRadians = position.in(Radians);
  }

  @Override
  public void setPosition(Angle angle) {
    setpoint = angle;
  }

  @Override
  public void stop() {
    setpoint = position;
  }
}
