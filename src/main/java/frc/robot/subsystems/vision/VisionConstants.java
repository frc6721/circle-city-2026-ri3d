// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {

  // TODO: tune
  public static final AprilTag TAG1 = new AprilTag(1, new Pose3d());
  public static final AprilTag TAG2 = new AprilTag(2, new Pose3d());
  public static final double FIELD_WIDTH_METERS = 8.23;
  public static final double FIELD_LENGTH_METERS = 16.46;

  // AprilTag layout
  // TODO: if your tags are setup to match the field, then use the field for this year's game
  // otherwise, update the tags to match your shop setup
  //   public static AprilTagFieldLayout aprilTagLayout =
  //       AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static AprilTagFieldLayout aprilTagLayout =
      new AprilTagFieldLayout(
          java.util.List.of(TAG1, TAG2), FIELD_LENGTH_METERS, FIELD_WIDTH_METERS);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  //   public static Transform3d robotToCamera1 =
  //       new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0 // Camera 0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
