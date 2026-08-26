package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Flywheel extends SubsystemBase {
  private static final double kHoodRollerRatio = 28.0 / 30.0;
  private static final double kBackRollerRatio = 30.0 / 30.0;

  private static final double kStallCurrentAmps = 68.0;
  private static final double kStallVelocityRPS = 2.0;

  private final TalonFX m_hoodRollerMotor;
  private final TalonFX m_backRollerMotor;

  private final VelocityVoltage m_hoodVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage m_backVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut m_hoodVoltageRequest = new VoltageOut(0);
  private final VoltageOut m_backVoltageRequest = new VoltageOut(0);

  private final Alert m_hoodRollerConnectionAlert = new Alert(
      "Shooter hood roller motor is not connected", AlertType.kWarning);
  private final Alert m_backRollerConnectionAlert = new Alert(
      "Shooter back roller motor is not connected", AlertType.kWarning);
  private final Alert m_hoodRollerStallAlert = new Alert(
      "Shooter hood roller motor is stalling", AlertType.kWarning);
  private final Alert m_backRollerStallAlert = new Alert(
      "Shooter back roller motor is stalling", AlertType.kWarning);

  private final Trigger m_isNearSetpointTrigger;

  private AngularVelocity m_setpoint = RPM.zero();

  public Flywheel(int hoodRollerMotorId, int backRollerMotorId) {
    m_hoodRollerMotor = new TalonFX(hoodRollerMotorId);
    m_backRollerMotor = new TalonFX(backRollerMotorId);

    configureMotor(m_hoodRollerMotor, kHoodRollerRatio);
    configureMotor(m_backRollerMotor, kBackRollerRatio);

    m_isNearSetpointTrigger = new Trigger(() -> isNearSetpoint(RPM.of(50)));
  }

  private static void configureMotor(TalonFX motor, double motorToRollerRatio) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = motorToRollerRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    AngularVelocity hoodVelocity = getHoodRollerVelocity();
    AngularVelocity backVelocity = getBackRollerVelocity();

    boolean hoodHighCurrent = m_hoodRollerMotor.getStatorCurrent().getValue().gt(Amps.of(kStallCurrentAmps));
    boolean hoodNotMoving = Math.abs(hoodVelocity.in(RotationsPerSecond)) < kStallVelocityRPS;
    boolean backHighCurrent = m_backRollerMotor.getStatorCurrent().getValue().gt(Amps.of(kStallCurrentAmps));
    boolean backNotMoving = Math.abs(backVelocity.in(RotationsPerSecond)) < kStallVelocityRPS;

    m_hoodRollerConnectionAlert.set(!m_hoodRollerMotor.isAlive());
    m_backRollerConnectionAlert.set(!m_backRollerMotor.isAlive());
    m_hoodRollerStallAlert.set(hoodHighCurrent && hoodNotMoving);
    m_backRollerStallAlert.set(backHighCurrent && backNotMoving);

    Logger.recordOutput("Flywheel/IsNearSetpoint", m_isNearSetpointTrigger.getAsBoolean());
  }

  @AutoLogOutput (key = "Flywheel/HoodRollerVelocity")
  public AngularVelocity getHoodRollerVelocity() {
    return m_hoodRollerMotor.getVelocity().getValue();
  }

  @AutoLogOutput (key = "Flywheel/BackRollerVelocity")
  public AngularVelocity getBackRollerVelocity() {
    return m_backRollerMotor.getVelocity().getValue();
  }

  @AutoLogOutput (key = "Flywheel/Setpoint")
  public AngularVelocity getSetpoint() {
    return m_setpoint;
  }

  public boolean isNearSetpoint(AngularVelocity tolerance) {
    return getHoodRollerVelocity().isNear(m_setpoint, tolerance)
        && getBackRollerVelocity().isNear(m_setpoint, tolerance);
  }
  
  public Trigger isNearSetpointTrigger() {
    return m_isNearSetpointTrigger;
  }

  public void setVelocity(AngularVelocity velocity) {
    m_setpoint = velocity;
    double rotationsPerSecond = velocity.in(RotationsPerSecond);
    m_hoodRollerMotor.setControl(m_hoodVelocityRequest.withVelocity(rotationsPerSecond));
    m_backRollerMotor.setControl(m_backVelocityRequest.withVelocity(rotationsPerSecond));
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return run(() -> setVelocity(velocity));
  }

  public void setVoltage(Voltage voltage) {
    m_setpoint = RPM.zero();
    m_hoodRollerMotor.setControl(m_hoodVoltageRequest.withOutput(voltage.in(Volts)));
    m_backRollerMotor.setControl(m_backVoltageRequest.withOutput(voltage.in(Volts)));
  }

  public Command setVoltageCommand(Voltage voltage) {
    return run(() -> setVoltage(voltage));
  }

  public void setPID(double kP, double kI, double kD, double kS, double kV) {
    Slot0Configs gains = new Slot0Configs()
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV);
    m_hoodRollerMotor.getConfigurator().apply(gains);
    m_backRollerMotor.getConfigurator().apply(gains);
  }

  public void stop() {
    m_setpoint = RPM.zero();
    m_hoodRollerMotor.stopMotor();
    m_backRollerMotor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
