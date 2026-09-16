package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.LoggedTunableNumber;
import frc.robot.constants.FieldConstants;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Camera {
  private static final int kMinimumBotposeLength = 11;
  private static final double kDefaultStdDevMultiplier = 2.0;
  private static final double kMaxObservationDistanceMeters = 7.62;
  private static final double kMaxObservationHeightMeters = 0.4572;
  private static final double kSimHorizontalFovRadians = Math.toRadians(82.0);
  private static final double kSimVerticalFovRadians = Math.toRadians(56.0);
  private static final double kSimLatencyMilliseconds = 30.0;

  private static final LoggedTunableNumber kCameraStdDevMultiplier =
      new LoggedTunableNumber("Vision/CameraStdDevMultiplier", 1.0);

  public record PoseObservation(
      double timestamp,
      Pose3d pose,
      int tagCount,
      double averageDistanceMeters,
      String cameraName,
      Matrix<N3, N1> standardDevs) {}

  private final String name;
  private final Transform3d robotToCamera;
  private final double stdDevMultiplier;
  private final Supplier<Boolean> enabledSupplier;
  private final Alert connectionAlert;
  private final DoubleArrayEntry botposeEntry;
  private final DoubleEntry heartbeatEntry;
  private final DoubleEntry pipelineEntry;
  private final DoubleEntry activePipelineEntry;

  private double lastHeartbeat = Double.NaN;
  private double lastHeartbeatChangeTimestamp;
  private boolean hasNewFrame;
  private int simulatedHeartbeat;

  public Camera(
      String name,
      Transform3d robotToCamera,
      double stdDevMultiplier,
      Supplier<Boolean> enabledSupplier) {
    this.name = name;
    this.robotToCamera = robotToCamera;
    this.stdDevMultiplier = stdDevMultiplier;
    this.enabledSupplier = enabledSupplier;
    connectionAlert = new Alert("Limelight " + name + " is not connected!", AlertType.kWarning);

    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    botposeEntry = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]);
    heartbeatEntry = table.getDoubleTopic("hb").getEntry(-1.0);
    pipelineEntry = table.getDoubleTopic("pipeline").getEntry(0.0);
    activePipelineEntry = table.getDoubleTopic("getpipe").getEntry(-1.0);

    publishCameraPose(table);
  }

  public Camera(String name, Transform3d robotToCamera, Supplier<Boolean> enabledSupplier) {
    this(name, robotToCamera, kDefaultStdDevMultiplier, enabledSupplier);
  }

  public Camera(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, kDefaultStdDevMultiplier, () -> true);
  }

  private void publishCameraPose(NetworkTable table) {
    Translation3d translation = robotToCamera.getTranslation();
    Rotation3d rotation = robotToCamera.getRotation();
    table
        .getDoubleArrayTopic("camerapose_robotspace_set")
        .publish()
        .set(
            new double[] {
              translation.getX(),
              translation.getY(),
              translation.getZ(),
              Math.toDegrees(rotation.getX()),
              Math.toDegrees(rotation.getY()),
              Math.toDegrees(rotation.getZ())
            });
  }

  public void periodic() {
    double heartbeat = heartbeatEntry.get();
    hasNewFrame = heartbeat >= 0.0 && heartbeat != lastHeartbeat;
    if (hasNewFrame) {
      lastHeartbeat = heartbeat;
      lastHeartbeatChangeTimestamp = Timer.getFPGATimestamp();
    }
    boolean connected =
        Timer.getFPGATimestamp() - lastHeartbeatChangeTimestamp < 1.0;
    Logger.recordOutput("Vision/" + name + "/Enabled", isEnabled());
    Logger.recordOutput("Vision/" + name + "/Connected", connected);
    connectionAlert.set(!connected);
  }

  private Matrix<N3, N1> calculateStandardDevs(int tagCount, double averageDistanceMeters) {
    double distanceMultiplier = Math.pow(averageDistanceMeters, 1.5);
    double standardDev =
        distanceMultiplier / tagCount * stdDevMultiplier * kCameraStdDevMultiplier.get();
    return VecBuilder.fill(standardDev, standardDev, standardDev);
  }

  public List<PoseObservation> getLatestObservations() {
    if (!hasNewFrame) {
      return List.of();
    }
    hasNewFrame = false;
    double[] values = botposeEntry.get();
    if (values.length < kMinimumBotposeLength) {
      return List.of();
    }

    int tagCount = (int) values[7];
    double averageDistance = values[9];
    Pose3d robotPose =
        new Pose3d(
            values[0],
            values[1],
            values[2],
            new Rotation3d(
                Math.toRadians(values[3]),
                Math.toRadians(values[4]),
                Math.toRadians(values[5])));

    Logger.recordOutput("Vision/" + name + "/TagCount", tagCount);
    Logger.recordOutput("Vision/" + name + "/AverageTagDistanceMeters", averageDistance);
    if (tagCount <= 0
        || Math.abs(robotPose.getZ()) > kMaxObservationHeightMeters
        || averageDistance > kMaxObservationDistanceMeters) {
      return List.of();
    }

    double timestamp = Timer.getFPGATimestamp() - values[6] / 1000.0;
    return List.of(
        new PoseObservation(
            timestamp,
            robotPose,
            tagCount,
            averageDistance,
            name,
            calculateStandardDevs(tagCount, averageDistance)));
  }

  public void simulationPeriodic(Pose2d robotPose) {
    Pose3d robotPose3d = new Pose3d(robotPose);
    Pose3d cameraPose = robotPose3d.transformBy(robotToCamera);
    var visibleTags =
        FieldConstants.kTagLayout.getTags().stream()
            .filter(
                tag -> {
                  Translation3d cameraToTag =
                      tag.pose.relativeTo(cameraPose).getTranslation();
                  if (cameraToTag.getX() <= 0.0) {
                    return false;
                  }
                  double distance = cameraToTag.getNorm();
                  double horizontalAngle =
                      Math.abs(Math.atan2(cameraToTag.getY(), cameraToTag.getX()));
                  double verticalAngle =
                      Math.abs(
                          Math.atan2(
                              cameraToTag.getZ(),
                              Math.hypot(cameraToTag.getX(), cameraToTag.getY())));
                  return distance <= kMaxObservationDistanceMeters
                      && horizontalAngle <= kSimHorizontalFovRadians / 2.0
                      && verticalAngle <= kSimVerticalFovRadians / 2.0;
                })
            .toList();

    double averageDistance =
        visibleTags.stream()
            .mapToDouble(tag -> tag.pose.getTranslation().getDistance(cameraPose.getTranslation()))
            .average()
            .orElse(0.0);
    Logger.recordOutput(
        "Vision/" + name + "/SimVisibleTagPoses",
        visibleTags.stream().map(tag -> tag.pose).toArray(Pose3d[]::new));
    Rotation3d rotation = robotPose3d.getRotation();
    botposeEntry.set(
        new double[] {
          robotPose3d.getX(),
          robotPose3d.getY(),
          robotPose3d.getZ(),
          Math.toDegrees(rotation.getX()),
          Math.toDegrees(rotation.getY()),
          Math.toDegrees(rotation.getZ()),
          kSimLatencyMilliseconds,
          visibleTags.size(),
          0.0,
          averageDistance,
          0.0
        });
    heartbeatEntry.set(simulatedHeartbeat++);
  }

  public String getName() {
    return name;
  }

  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  public boolean isEnabled() {
    return enabledSupplier.get();
  }

  public int getPipelineIndex() {
    return (int) activePipelineEntry.get();
  }

  public void setPipelineIndex(int index) {
    pipelineEntry.set(index);
  }
}
