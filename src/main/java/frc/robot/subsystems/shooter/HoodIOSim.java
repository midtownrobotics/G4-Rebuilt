package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public class HoodIOSim implements HoodIO {
  private Angle position = Hood.kMinimumAngle, setpoint = Hood.kMinimumAngle;
  private double previous = position.in(Radians);

  @Override
  public void updateInputs(HoodIOInputs x) {
    double now = position.in(Radians),
        error = setpoint.in(Radians) - now,
        step = Math.max(-Math.toRadians(45) * .02, Math.min(Math.toRadians(45) * .02, error));
    position = Radians.of(now + step);
    x.position = position;
    x.absolutePosition = position;
    x.velocity = RadiansPerSecond.of((position.in(Radians) - previous) / .02);
    x.setpoint = setpoint;
    x.motorConnected = true;
    x.encoderConnected = true;
    previous = position.in(Radians);
  }

  @Override
  public void setPosition(Angle a) {
    setpoint = a;
  }

  @Override
  public void setEncoderPosition(Angle a) {
    position = a;
    setpoint = a;
  }

  @Override
  public void stop() {
    setpoint = position;
  }
}
