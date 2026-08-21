package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("YawPosition", yawPosition);
    table.put("PitchPosition", pitchPosition);
    table.put("RollPosition", rollPosition);
    table.put("OdometryTimestamps", odometryTimestamps);
    table.put("OdometryYawPositions", odometryYawPositions);
    table.put("OdometryPitchPositions", odometryPitchPositions);
    table.put("OdometryRollPositions", odometryRollPositions);
    table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
    table.put("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
    table.put("RollVelocityRadPerSec", rollVelocityRadPerSec);
    table.put("XAcceleration", xAcceleration);
    table.put("YAcceleration", yAcceleration);
    table.put("ZAcceleration", zAcceleration);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yawPosition = table.get("YawPosition", yawPosition);
    pitchPosition = table.get("PitchPosition", pitchPosition);
    rollPosition = table.get("RollPosition", rollPosition);
    odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
    odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
    odometryPitchPositions = table.get("OdometryPitchPositions", odometryPitchPositions);
    odometryRollPositions = table.get("OdometryRollPositions", odometryRollPositions);
    yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
    pitchVelocityRadPerSec = table.get("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
    rollVelocityRadPerSec = table.get("RollVelocityRadPerSec", rollVelocityRadPerSec);
    xAcceleration = table.get("XAcceleration", xAcceleration);
    yAcceleration = table.get("YAcceleration", yAcceleration);
    zAcceleration = table.get("ZAcceleration", zAcceleration);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yawPosition = this.yawPosition;
    copy.pitchPosition = this.pitchPosition;
    copy.rollPosition = this.rollPosition;
    copy.odometryTimestamps = this.odometryTimestamps.clone();
    copy.odometryYawPositions = this.odometryYawPositions.clone();
    copy.odometryPitchPositions = this.odometryPitchPositions.clone();
    copy.odometryRollPositions = this.odometryRollPositions.clone();
    copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
    copy.pitchVelocityRadPerSec = this.pitchVelocityRadPerSec;
    copy.rollVelocityRadPerSec = this.rollVelocityRadPerSec;
    copy.xAcceleration = this.xAcceleration;
    copy.yAcceleration = this.yAcceleration;
    copy.zAcceleration = this.zAcceleration;
    return copy;
  }
}
