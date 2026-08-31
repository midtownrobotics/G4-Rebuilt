package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();
  private final Alert connectionAlert =
      new Alert("IntakeRoller TalonFX motor is not connected", AlertType.kWarning);
  private final Alert stallAlert = new Alert("IntakeRoller is stalling", AlertType.kWarning);

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeRoller", inputs);
    connectionAlert.set(!inputs.motorConnected);
    stallAlert.set(inputs.statorCurrent.gt(Amps.of(68)) && inputs.velocity.abs(RPM) < 120.0);
  }

  public void setVoltage(Voltage voltage) {
    io.setVoltage(voltage);
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> setVoltage(voltage));
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
