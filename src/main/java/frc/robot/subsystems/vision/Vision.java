// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * The Vision subsystem manages camera-based positioning using AprilTags.
 *
 * <p><b>What is Vision Positioning?</b> AprilTags are special markers placed at known locations on
 * the field. Cameras on the robot detect these tags and calculate the robot's position. This helps
 * correct odometry drift caused by wheel slippage.
 *
 * <p><b>Hardware Overview:</b>
 *
 * <ul>
 *   <li>Tested with Limelight vision processor
 *   <li>Can support multiple cameras simultaneously
 *   <li>Each camera independently processes AprilTags and provides pose estimates
 * </ul>
 *
 * <p><b>How It Works:</b>
 *
 * <ol>
 *   <li>Camera(s) capture images and detect AprilTags
 *   <li>Vision processor calculates robot position based on tag locations
 *   <li>Vision subsystem receives pose estimates with timestamps
 *   <li>Subsystem validates estimates (checks for realistic values)
 *   <li>Accepted estimates are sent to Drive subsystem's pose estimator
 *   <li>Pose estimator fuses vision data with wheel odometry
 * </ol>
 *
 * <p><b>Pose Validation:</b> Not all vision measurements are trustworthy. This subsystem rejects
 * poses that:
 *
 * <ul>
 *   <li>Have no detected tags (tagCount == 0)
 *   <li>Have only one tag with high ambiguity (uncertain detection)
 *   <li>Report unrealistic Z coordinates (height off the ground)
 *   <li>Fall outside the field boundaries
 * </ul>
 *
 * <p><b>Standard Deviation Calculation:</b> Each vision measurement includes standard deviations
 * that tell the pose estimator how much to trust it:
 *
 * <ul>
 *   <li><b>Closer tags = smaller stddev = more trust:</b> Tags nearby are easier to measure
 *       accurately
 *   <li><b>More tags = smaller stddev = more trust:</b> Multiple tags provide redundancy
 *   <li><b>MegaTag2 = smaller stddev = more trust:</b> Advanced algorithm for better accuracy
 *   <li><b>Per-camera factors:</b> Some cameras may be more/less reliable
 * </ul>
 *
 * <p><b>Status for RI3D:</b> Vision was tested with Limelight but NOT fully implemented in the
 * competition robot. The infrastructure is here and working, but the camera and offsets where not
 * fully tuned.
 *
 * <p>This subsystem uses the AdvantageKit IO layer pattern and supports multiple cameras.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  /**
   * Creates a new Vision subsystem.
   *
   * <p>This constructor sets up vision processing with support for multiple cameras. Each camera
   * independently provides pose estimates that are validated and sent to the consumer (typically
   * the Drive subsystem's pose estimator).
   *
   * <p><b>VisionConsumer:</b> This is a callback function that receives validated vision
   * measurements. When you create the Vision subsystem in RobotContainer, you pass a reference to
   * the Drive subsystem's addVisionMeasurement() method:
   *
   * <pre>
   * Vision vision = new Vision(
   *   drive::addVisionMeasurement,  // Consumer receives measurements
   *   new VisionIOLimelight()         // One or more camera IOs
   * );
   * </pre>
   *
   * <p><b>Multiple Cameras:</b> You can pass multiple VisionIO objects to use multiple cameras.
   *
   * @param consumer Callback function to receive validated vision measurements (typically
   *     drive::addVisionMeasurement)
   * @param io One or more VisionIO implementations, one per camera
   */
  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target for simple vision servoing.
   *
   * <p>This provides a simpler alternative to full pose estimation. Instead of calculating the
   * robot's position on the field, this just tells you how far left or right the target appears in
   * the camera view.
   *
   * <p><b>Use Cases:</b>
   *
   * <ul>
   *   <li>Aligning to a target without full odometry
   *   <li>Simple "turn toward target" commands
   *   <li>Quick alignment during teleop
   * </ul>
   *
   * <p><b>How to use:</b>
   *
   * <pre>
   * Rotation2d angle = vision.getTargetX(0);
   * if (Math.abs(angle.getDegrees()) < 2.0) {
   *   // Aligned! Ready to shoot
   * } else {
   *   // Turn toward target
   *   drive.turnToAngle(angle);
   * }
   * </pre>
   *
   * <p><b>Note:</b> This is simpler than pose estimation but less accurate for autonomous
   * navigation. Good for quick teleop alignment.
   *
   * @param cameraIndex The index of the camera to use (0 for first camera, 1 for second, etc.)
   * @return The horizontal angle to the target (positive = target is to the right)
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  /**
   * Periodic method called every 20 milliseconds (50 times per second).
   *
   * <p>This is the main vision processing loop. For each camera:
   *
   * <ol>
   *   <li><b>Read sensor data:</b> Get pose observations from each camera
   *   <li><b>Update disconnection alerts:</b> Warn if cameras go offline
   *   <li><b>Validate poses:</b> Reject unrealistic or low-quality measurements
   *   <li><b>Calculate standard deviations:</b> Determine how much to trust each measurement
   *   <li><b>Send to consumer:</b> Pass validated measurements to Drive subsystem
   *   <li><b>Log everything:</b> Record all data for AdvantageScope visualization
   * </ol>
   *
   * <p><b>Validation Checks:</b> Poses are rejected if they:
   *
   * <ul>
   *   <li>Have no detected tags
   *   <li>Have only one tag with high ambiguity (uncertain)
   *   <li>Report unrealistic Z coordinate (too high/low off ground)
   *   <li>Fall outside the field boundaries
   * </ul>
   *
   * <p><b>Standard Deviation Factors:</b> Trust decreases (stddev increases) with:
   *
   * <ul>
   *   <li>Distance to tags (square of distance)
   *   <li>Fewer tags detected (dividing by tag count)
   *   <li>Standard pose estimation vs. MegaTag2
   *   <li>Per-camera reliability factors
   * </ul>
   *
   * <p><b>Logging:</b> All poses (accepted and rejected), tag positions, and metadata are logged
   * for each camera and in summary views. You can visualize these in AdvantageScope on a 3D field
   * view to see what the vision system "sees".
   */
  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  /**
   * Functional interface for consuming vision measurements.
   *
   * <p>This defines the signature for a callback function that receives validated vision
   * measurements. The Drive subsystem's addVisionMeasurement() method implements this interface.
   *
   * <p><b>Why use an interface?</b> This decouples the Vision subsystem from the Drive subsystem.
   * Vision doesn't need to know about Drive - it just calls whatever function you give it. This
   * makes testing easier and the code more flexible.
   *
   * <p><b>Functional Interface:</b> This annotation means the interface has exactly one method,
   * allowing it to be used with lambda expressions or method references:
   *
   * <pre>
   * // Method reference (clean!)
   * new Vision(drive::addVisionMeasurement, visionIO);
   *
   * // Lambda (equivalent but more verbose)
   * new Vision((pose, time, stdDevs) -> drive.addVisionMeasurement(pose, time, stdDevs), visionIO);
   * </pre>
   *
   * <p>This is an advanced Java feature that makes the code cleaner while maintaining flexibility.
   */
  @FunctionalInterface
  public static interface VisionConsumer {
    /**
     * Accepts a vision measurement for pose estimation.
     *
     * <p>This method signature matches Drive.addVisionMeasurement(), allowing it to be used as a
     * method reference.
     *
     * @param visionRobotPoseMeters The robot pose calculated from vision (2D position + rotation)
     * @param timestampSeconds When the measurement was captured (seconds since robot start)
     * @param visionMeasurementStdDevs Standard deviations for x, y, and rotation (how much to trust
     *     this measurement)
     */
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
