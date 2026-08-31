package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final LoggedTunableNumber p = new LoggedTunableNumber("Flywheel/kP", 0),
      i = new LoggedTunableNumber("Flywheel/kI", 0),
      d = new LoggedTunableNumber("Flywheel/kD", 0),
      s = new LoggedTunableNumber("Flywheel/kS", 0),
      v = new LoggedTunableNumber("Flywheel/kV", 0);
  private final Alert hoodAlert = new Alert("Shooter hood roller disconnected", AlertType.kWarning),
      backAlert = new Alert("Shooter back roller disconnected", AlertType.kWarning),
      hoodStall = new Alert("Shooter hood roller stalling", AlertType.kWarning),
      backStall = new Alert("Shooter back roller stalling", AlertType.kWarning);
  private final Trigger near = new Trigger(() -> isNearSetpoint(RPM.of(50)));

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    LoggedTunableNumber.ifChanged(
        hashCode(), x -> io.setPID(x[0], x[1], x[2], x[3], x[4]), p, i, d, s, v);
    hoodAlert.set(!inputs.hoodConnected);
    backAlert.set(!inputs.backConnected);
    hoodStall.set(
        inputs.hoodStatorCurrent.gt(Amps.of(68))
            && inputs.hoodVelocity.abs(RotationsPerSecond) < 2);
    backStall.set(
        inputs.backStatorCurrent.gt(Amps.of(68))
            && inputs.backVelocity.abs(RotationsPerSecond) < 2);
    Logger.recordOutput("Flywheel/IsNearSetpoint", near.getAsBoolean());
  }

  public AngularVelocity getHoodRollerVelocity() {
    return inputs.hoodVelocity;
  }

  public AngularVelocity getBackRollerVelocity() {
    return inputs.backVelocity;
  }

  public boolean isNearSetpoint(AngularVelocity tolerance) {
    return inputs.hoodVelocity.isNear(inputs.setpoint, tolerance)
        && inputs.backVelocity.isNear(inputs.setpoint, tolerance);
  }

  public Trigger isNearSetpointTrigger() {
    return near;
  }

  public void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return run(() -> io.setVelocity(velocity));
  }

  public void setVoltage(Voltage voltage) {
    io.setVoltage(voltage);
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> io.setVoltage(voltage));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(io::stop);
  }
}
