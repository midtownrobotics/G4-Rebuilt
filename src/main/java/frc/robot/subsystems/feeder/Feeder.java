package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Feeder/kP", 0.1);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Feeder/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Feeder/kD", 0.0);
  private final Alert leaderAlert = new Alert("Feeder leader disconnected", AlertType.kWarning);
  private final Alert followerAlert = new Alert("Feeder follower disconnected", AlertType.kWarning);
  private final Alert stallAlert = new Alert("Feeder is stalling", AlertType.kWarning);

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    LoggedTunableNumber.ifChanged(hashCode(), v -> io.setPID(v[0], v[1], v[2]), kP, kI, kD);
    leaderAlert.set(!inputs.leaderConnected);
    followerAlert.set(!inputs.followerConnected);
    stallAlert.set(
        inputs.leaderStatorCurrent.gt(Amps.of(30)) && inputs.leaderVelocity.abs(RPM) < 120);
  }

  public void setVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity);
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return run(() -> io.setVelocity(velocity)).finallyDo(io::stop);
  }

  public void setVoltage(Voltage voltage) {
    io.setVoltage(voltage);
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> io.setVoltage(voltage)).finallyDo(io::stop);
  }

  public Command runForwardCommand() {
    return setVoltageCommand(Volts.of(10));
  }

  public Command runReverseCommand() {
    return setVoltageCommand(Volts.of(-10));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(io::stop);
  }
}
