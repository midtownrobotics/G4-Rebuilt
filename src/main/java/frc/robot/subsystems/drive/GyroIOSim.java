package frc.robot.subsystems.drive;

/** Uses wheel kinematics as the simulated heading source. */
public class GyroIOSim implements GyroIO {
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = false;
  }
}
