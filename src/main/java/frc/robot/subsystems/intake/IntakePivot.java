package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Intake pivot logic. Hardware and simulation details live behind {@link IntakePivotIO}. */
public class IntakePivot extends SubsystemBase {
  public enum State {
    START(Degrees.of(90)),
    INTAKING(Degrees.of(0)),
    NOT_INTAKING(Degrees.of(90));

    private final Angle angle;

    State(Angle angle) {
      this.angle = angle;
    }

    public Angle angle() {
      return angle;
    }
  }

  private final IntakePivotIO io;
  private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private final Alert motorAlert =
      new Alert("IntakePivot TalonFX motor is not connected", AlertType.kWarning);
  private final Alert encoderAlert =
      new Alert("IntakePivot CANcoder is not connected", AlertType.kWarning);
  private final Alert stallAlert = new Alert("IntakePivot is stalling", AlertType.kWarning);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("IntakePivot/kP", 0.0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("IntakePivot/kI", 0.0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("IntakePivot/kD", 0.0);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("IntakePivot/kS", 0.0);
  private final LoggedTunableNumber kG = new LoggedTunableNumber("IntakePivot/kG", 0.0);

  private State state = State.START;

  public IntakePivot(IntakePivotIO io) {
    this.io = io;
    setState(State.START);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakePivot", inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> io.setPID(values[0], values[1], values[2], values[3], values[4]),
        kP,
        kI,
        kD,
        kS,
        kG);

    double filteredCurrent = currentFilter.calculate(inputs.statorCurrent.in(Amps));
    Logger.recordOutput("IntakePivot/State", state);
    Logger.recordOutput("IntakePivot/CurrentSpike", filteredCurrent > 20.0);
    motorAlert.set(!inputs.motorConnected);
    encoderAlert.set(!inputs.encoderConnected);
    stallAlert.set(inputs.statorCurrent.gt(Amps.of(30)));
  }

  public void setAngle(Angle angle) {
    io.setPosition(angle);
  }

  public void setState(State state) {
    this.state = state;
    setAngle(state.angle());
  }

  public Command setStateCommand(State state) {
    return runOnce(() -> setState(state));
  }

  public Command setAngleCommand(Angle angle) {
    return runOnce(() -> setAngle(angle));
  }

  public Angle getAngle() {
    return inputs.position;
  }

  public void start() {
    setState(State.START);
  }

  public void stop() {
    io.stop();
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, this);
  }
}
