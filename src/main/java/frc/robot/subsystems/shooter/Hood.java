package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  public static final Angle kMinimumAngle = Degrees.of(16.25), kMaximumAngle = Degrees.of(35);
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final LinearFilter filter = LinearFilter.movingAverage(5);
  private final LoggedTunableNumber p = new LoggedTunableNumber("Hood/kP", 800),
      i = new LoggedTunableNumber("Hood/kI", 0),
      d = new LoggedTunableNumber("Hood/kD", 18),
      s = new LoggedTunableNumber("Hood/kS", .395),
      v = new LoggedTunableNumber("Hood/kV", 30),
      g = new LoggedTunableNumber("Hood/kG", .015);
  private final Alert motorAlert = new Alert("Hood TalonFX motor disconnected", AlertType.kWarning),
      encoderAlert = new Alert("Hood CANcoder disconnected", AlertType.kWarning),
      stallAlert = new Alert("Hood is stalling", AlertType.kWarning);
  private final Trigger near = new Trigger(() -> isNearSetpoint(Degrees.of(1)));

  public Hood(HoodIO io) {
    this.io = io;
    io.setPosition(kMinimumAngle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    LoggedTunableNumber.ifChanged(
        hashCode(), x -> io.setPID(x[0], x[1], x[2], x[3], x[4], x[5]), p, i, d, s, v, g);
    double current = filter.calculate(inputs.statorCurrent.in(Amps));
    Logger.recordOutput("Hood/CurrentSpike", current > 20);
    Logger.recordOutput("Hood/IsNearSetpoint", near.getAsBoolean());
    motorAlert.set(!inputs.motorConnected);
    encoderAlert.set(!inputs.encoderConnected);
    stallAlert.set(inputs.statorCurrent.gt(Amps.of(30)));
  }

  public Angle getAngle() {
    return inputs.position;
  }

  public Angle getAbsoluteAngle() {
    return inputs.absolutePosition;
  }

  public Angle getAngleFromMinimum() {
    return getAngle().minus(kMinimumAngle);
  }

  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  public boolean isNearSetpoint(Angle tolerance) {
    return inputs.position.isNear(inputs.setpoint, tolerance);
  }

  public Trigger isNearSetpointTrigger() {
    return near;
  }

  public Trigger isNearTrigger(Supplier<Angle> a, Angle t) {
    return new Trigger(() -> getAngle().isNear(a.get(), t));
  }

  public void setAngle(Angle angle) {
    io.setPosition(angle);
  }

  public Command setAngleCommand(Angle angle) {
    return runOnce(() -> io.setPosition(angle));
  }

  public Command setAngleCommand(Supplier<Angle> a) {
    return run(() -> io.setPosition(a.get()));
  }

  public Command setMinimumAngleCommand() {
    return setAngleCommand(kMinimumAngle);
  }

  public Command setMaximumAngleCommand() {
    return setAngleCommand(kMaximumAngle);
  }

  public void setVoltage(Voltage voltage) {
    io.setVoltage(voltage);
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> io.setVoltage(voltage));
  }

  public void setEncoderPosition(Angle angle) {
    io.setEncoderPosition(angle);
  }

  public Command zeroEncoderCommand() {
    return runOnce(() -> io.setEncoderPosition(Degrees.zero()));
  }

  public void setLowerSoftLimitEnabled(boolean enabled) {
    io.setLowerSoftLimitEnabled(enabled);
  }

  public Command setLowerSoftLimitEnabledCommand(boolean enabled) {
    return runOnce(() -> io.setLowerSoftLimitEnabled(enabled));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(io::stop);
  }
}
