package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class IntakePivot {
    private final TalonFX intakePivotMotor;
    private final CANcoder intakePivotEncoder;
    private final TalonFXConfiguration config;

    public enum State {
        START(Degrees.of(115)),
        INTAKING(Degrees.of(0)),
        NOTINTAKING(Degrees.of(90));
    }

    public IntakePivot(int intakePivotMotorId, int encoderId) {
        intakePivotMotor = new TalonFX(intakePivotMotorId);
        intakePivotEncoder = new CANcoder(encoderId);
        
        config = new TalonFXConfiguration();
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = 0.0237; // Constants file imports being weird, temp hardcode
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.CurrentLimits.StatorCurrentLimit = Amps.of(120);
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Amps.of(70);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Rotations.fromBaseUnits(Degrees.toBaseUnits(90));
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;


    }

    public setAngle(Angle newAngle) {
        intakePivotMotor.setPosition(newAngle);
    }

    public double getAngle() {
        return intakePivotEncoder.get() * kIntakePivotReduction;
    }

    public void start() {
        setAngle();
    }

}
