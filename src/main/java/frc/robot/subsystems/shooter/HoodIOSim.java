package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;

public class HoodIOSim implements HoodIO {
  private Angle position = Hood.kMinimumAngle, setpoint = Hood.kMinimumAngle;
  private double previous = position.in(Radians);
  private double appliedVolts;
  private boolean voltageControl;

  @Override
  public void updateInputs(HoodIOInputs x) {
    double now = position.in(Radians);
    double maxStep = Math.toRadians(45) * .02;
    double step =
        voltageControl
            ? maxStep * appliedVolts / 12.0
            : MathUtil.clamp(setpoint.in(Radians) - now, -maxStep, maxStep);
    position =
        Radians.of(
            MathUtil.clamp(
                now + step,
                Hood.kMinimumAngle.in(Radians),
                Hood.kMaximumAngle.in(Radians)));
    x.position = position;
    x.absolutePosition = position;
    x.velocity = RadiansPerSecond.of((position.in(Radians) - previous) / .02);
    x.appliedVoltage = Volts.of(appliedVolts);
    x.setpoint = setpoint;
    x.motorConnected = true;
    x.encoderConnected = true;
    previous = position.in(Radians);
  }

  @Override
  public void setPosition(Angle a) {
    voltageControl = false;
    appliedVolts = 0.0;
    setpoint = a;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    voltageControl = true;
    appliedVolts = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
  }

  @Override
  public void setEncoderPosition(Angle a) {
    voltageControl = false;
    appliedVolts = 0.0;
    position = a;
    setpoint = a;
  }

  @Override
  public void stop() {
    voltageControl = false;
    appliedVolts = 0.0;
    setpoint = position;
  }
}
