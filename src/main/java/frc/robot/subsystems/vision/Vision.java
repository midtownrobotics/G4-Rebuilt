package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.Camera.PoseObservation;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Vision extends SubsystemBase {
  private static final double kMaxDistanceFromFusedPoseMeters = 4.0;
  private static final double kVisionTimeoutSeconds = 1.0;

  private final List<Camera> cameras;
  private final Consumer<PoseObservation> addVisionMeasurement;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Pose2d> resetPoseConsumer;
  private final TimeInterpolatableBuffer<Pose2d> acceptedObservations =
      TimeInterpolatableBuffer.createBuffer(0.1);
  private final LoggedNetworkBoolean enableVisionObservations =
      new LoggedNetworkBoolean("Toggles/UseVisionObservations", true);

  private boolean hasVisionUpdate;
  private boolean hasAcceptedVisionUpdate;
  private double lastAcceptedVisionTimestamp;

  private final Trigger hasVisionUpdateTrigger = new Trigger(this::hasVisionUpdate);
  private final Trigger hasAcceptedVisionUpdateTrigger = new Trigger(this::hasAcceptedVisionUpdate);
  private final Trigger hasRecentAcceptedVisionTrigger = new Trigger(this::hasRecentAcceptedVision);

  public Vision(
      Consumer<PoseObservation> addVisionMeasurement,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPoseConsumer,
      Camera... cameras) {
    this.cameras = List.of(cameras);
    this.addVisionMeasurement = addVisionMeasurement;
    this.poseSupplier = poseSupplier;
    this.resetPoseConsumer = resetPoseConsumer;

    SmartDashboard.putData(
        "Commands/Vision/ResetRobotPoseToLatestVisionPose",
        resetRobotPoseToLatestVisionPoseCommand());
  }

  @Override
  public void periodic() {
    Pose2d robotPose = poseSupplier.get();
    Pose3d robotPose3d = new Pose3d(robotPose);
    Logger.recordOutput(
        "Vision/CameraPoses",
        cameras.stream()
            .map(camera -> robotPose3d.transformBy(camera.getRobotToCamera()))
            .toArray(Pose3d[]::new));

    for (Camera camera : cameras) {
      camera.periodic();
      Logger.recordOutput(
          "Vision/" + camera.getName() + "/CameraPose",
          robotPose3d.transformBy(camera.getRobotToCamera()));
    }

    List<PoseObservation> observations =
        cameras.stream()
            .filter(Camera::isEnabled)
            .flatMap(camera -> camera.getLatestObservations().stream())
            .sorted((a, b) -> Double.compare(a.timestamp(), b.timestamp()))
            .toList();

    hasVisionUpdate = !observations.isEmpty();
    hasAcceptedVisionUpdate = false;
    boolean poseTrusted = hasRecentAcceptedVision();

    for (PoseObservation observation : observations) {
      Logger.recordOutput(
          "Vision/" + observation.cameraName() + "/ObservedRobotPose", observation.pose());
      double distanceFromFusedPose =
          robotPose.getTranslation().getDistance(observation.pose().toPose2d().getTranslation());
      Logger.recordOutput(
          "Vision/" + observation.cameraName() + "/DistanceFromFusedPose",
          distanceFromFusedPose);

      if (poseTrusted && distanceFromFusedPose > kMaxDistanceFromFusedPoseMeters) {
        continue;
      }

      acceptedObservations.addSample(observation.timestamp(), observation.pose().toPose2d());
      hasAcceptedVisionUpdate = true;
      lastAcceptedVisionTimestamp = Timer.getFPGATimestamp();
      if (enableVisionObservations.get()) {
        addVisionMeasurement.accept(observation);
      }
    }

    Logger.recordOutput("Vision/ObservationCount", observations.size());
    Logger.recordOutput("Vision/HasVisionUpdate", hasVisionUpdate);
    Logger.recordOutput("Vision/HasAcceptedVisionUpdate", hasAcceptedVisionUpdate);
    Logger.recordOutput("Vision/HasRecentAcceptedVision", hasRecentAcceptedVision());
    Logger.recordOutput("Vision/ArePipelinesReady", areAllPipelinesReady());
  }

  public Optional<Pose2d> getPoseAtTime(double timestamp) {
    return acceptedObservations.getSample(timestamp);
  }

  @Override
  public void simulationPeriodic() {
    Pose2d robotPose = poseSupplier.get();
    cameras.forEach(camera -> camera.simulationPeriodic(robotPose));
  }

  public boolean hasVisionUpdate() {
    return hasVisionUpdate;
  }

  public Trigger getHasVisionUpdateTrigger() {
    return hasVisionUpdateTrigger;
  }

  public boolean hasAcceptedVisionUpdate() {
    return hasAcceptedVisionUpdate;
  }

  public Trigger getHasAcceptedVisionUpdateTrigger() {
    return hasAcceptedVisionUpdateTrigger;
  }

  public boolean hasRecentAcceptedVision() {
    return Timer.getFPGATimestamp() - lastAcceptedVisionTimestamp < kVisionTimeoutSeconds;
  }

  public Trigger getHasRecentAcceptedVisionTrigger() {
    return hasRecentAcceptedVisionTrigger;
  }

  public Command resetRobotPoseToLatestVisionPoseCommand() {
    return Commands.runOnce(
        () -> {
          if (!acceptedObservations.getInternalBuffer().isEmpty()) {
            resetPoseConsumer.accept(acceptedObservations.getInternalBuffer().lastEntry().getValue());
          }
        });
  }

  public boolean areAllPipelinesReady() {
    return cameras.stream().allMatch(camera -> camera.getPipelineIndex() == 0);
  }

  public void setPipelinesToIndex(int index) {
    cameras.forEach(camera -> camera.setPipelineIndex(index));
  }
}
