package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants {
  private VisionConstants() {}

  public static final String kCameraName = "limelight";

  public static final Transform3d kRobotToCamera =
      new Transform3d(
          new Translation3d(Inches.of(-13.920), Inches.zero(), Inches.of(13.122)),
          new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(180.0)));
}
