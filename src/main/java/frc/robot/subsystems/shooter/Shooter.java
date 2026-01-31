// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.VirtualHopper;
import frc.robot.subsystems.shooter.io.ShooterIO;
import frc.robot.subsystems.shooter.io.ShooterIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The Shooter subsystem controls the robot's game piece launching mechanism.
 *
 * <p><b>Hardware Overview:</b>
 *
 * <ul>
 *   <li>Based on the AndyMark "Launcher in a Box" design
 *   <li>Modified to fit under 22" trench (top 3" removed)
 *   <li>Single NEO motor driving the flywheel 1:1 (direct drive)
 *   <li>4" Solid Urethane Wheel from ThriftyBot as the shooting wheel
 *   <li>Custom ½" thick aluminum flywheel for momentum and consistency
 *   <li>Supported by 1x1 (1/16" wall) aluminum tubing frame
 *   <li>Polycarbonate backing to guide game pieces into the wheel
 * </ul>
 *
 * <p><b>How It Works:</b>
 *
 * <ul>
 *   <li><b>Flywheel:</b> A heavy wheel that spins at high speed to store rotational energy
 *   <li><b>Launch:</b> When a game piece contacts the spinning flywheel, it's launched at high
 *       speed
 *   <li><b>Distance Control:</b> Different flywheel speeds = different launch distances
 *   <li><b>Consistency:</b> The heavy flywheel maintains speed between shots for repeatable
 *       performance
 * </ul>
 *
 * <p><b>Software Features:</b>
 *
 * <ul>
 *   <li>Closed-loop (PID) velocity control for precise flywheel speed
 *   <li>Distance-based shooting using WPILib's InterpolatingDoubleTreeMap
 *   <li>InterpolatingDoubleTreeMap stores (distance → RPM) data points in ShooterConstants
 *   <li>Automatic linear interpolation for distances between data points
 *   <li>Integration with Drive subsystem to get distance to target
 *   <li>Comprehensive logging of RPM, setpoints, currents, and calculated speeds
 * </ul>
 *
 * <p><b>Distance-Based Shooting Concept:</b>
 *
 * <ol>
 *   <li>Robot calculates distance to target (e.g., the Hub) using odometry
 *   <li>getSpeedForDistance() looks up required RPM from the InterpolatingDoubleTreeMap
 *   <li>Map automatically interpolates between your characterization data points
 *   <li>Shooter spins up to calculated RPM
 *   <li>Once at target speed, feeder automatically feeds the game piece
 * </ol>
 *
 * <p><b>Key Learnings from RI3D:</b>
 *
 * <ul>
 *   <li>Flywheels are critical for shot consistency - the heavier the better (up to a point)
 *   <li>Shape the polycarbonate backing carefully for consistent feed angle
 *   <li>Start with simple designs and test extensively before adding complexity
 *   <li>InterpolatingDoubleTreeMap makes distance-based shooting simple and reliable
 *   <li>Use the units library when adding data points for clarity and to prevent unit errors
 * </ul>
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO _shooterIO;
  private final ShooterIOInputsAutoLogged _shooterInputs = new ShooterIOInputsAutoLogged();
  private AngularVelocity _targetFlywheelSpeed;
  private final SysIdRoutine sysId;

  private final FuelVisualizer _fuelSimVisualizer;

  /**
   * Creates a new Shooter subsystem.
   *
   * <p>Initializes the shooter with the provided hardware IO and stops the flywheel for safety. The
   * flywheel will remain stopped until commanded to spin.
   *
   * <p><b>RobotState Integration:</b> This subsystem uses the centralized RobotState singleton to
   * access robot pose and velocity. This decouples the Shooter from the Drive subsystem and
   * simplifies the constructor.
   *
   * @param shooterIO The hardware interface for shooter control (motor and sensors)
   */
  public Shooter(ShooterIO shooterIO) {
    this._shooterIO = shooterIO;
    this.stopFlywheels();

    // Initialize FuelSim visualizer for trajectory and launch simulation
    _fuelSimVisualizer = new FuelVisualizer();

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysId/State", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage), null, this));
  }

  /**
   * Periodic method called every 20 milliseconds (50 times per second).
   *
   * <p>This method handles:
   *
   * <ul>
   *   <li>Reading sensor data from the shooter hardware (flywheel velocity, current, etc.)
   *   <li>Logging all sensor data and setpoints to AdvantageKit
   *   <li>Checking if the flywheel has reached its target speed
   *   <li>Updating the visualization
   * </ul>
   *
   * <p><b>What gets logged:</b>
   *
   * <ul>
   *   <li>Current flywheel speed (in rad/s and RPM)
   *   <li>Desired flywheel speed (in rad/s and RPM)
   *   <li>Whether the flywheel is at target speed (boolean)
   *   <li>Motor current and voltage (via IO layer)
   * </ul>
   *
   * <p>The actual velocity control happens in the hardware layer (RealShooterIO) using the motor
   * controller's built-in PID.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _shooterIO.updateInputs(_shooterInputs);
    Logger.processInputs("Shooter", _shooterInputs);

    // LOGGING
    Logger.recordOutput(
        "Shooter/FlywheelSpeed/Current_RadPerSec",
        _shooterInputs._flywheelMotorVelocity.in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/FlywheelSpeed/Current_RPM",
        _shooterInputs._flywheelMotorVelocity.in(RevolutionsPerSecond) * 60);

    Logger.recordOutput(
        "Shooter/FlywheelSpeed/Desired_RadPerSec", _targetFlywheelSpeed.in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/FlywheelSpeed/Desired_RPM", _targetFlywheelSpeed.in(RevolutionsPerSecond) * 60);
    Logger.recordOutput("Shooter/AtTargetSpeed", this.areFlywheelsAtTargetSpeed());

    // ==================== FUEL SIM INTEGRATION ====================
    // Update trajectory visualization every loop so driver sees real-time prediction
    LinearVelocity linearSpeed =
        _fuelSimVisualizer.convertToLinearVelocity(_shooterInputs._flywheelMotorVelocity);
    _fuelSimVisualizer.updateTrajectory(linearSpeed, ShooterConstants.SHOOTER_HOOD_ANGLE);

    // Check if we should launch fuel (for visualization)
    Logger.recordOutput("Shooter/FuelSim/ShouldLaunch", shouldVisualizeLaunch());

    // Actually launch fuel if conditions are met
    visualizeFuelLaunch();
  }

  // ==================== FUEL SIM METHODS ====================

  /**
   * Determines if fuel should be visualized as launching.
   *
   * <p>This is for VISUALIZATION only and does NOT affect actual robot shooting. The driver still
   * controls shooting with existing buttons.
   *
   * <p>All conditions must be true:
   *
   * <ul>
   *   <li>Flywheels are at target speed (within tolerance)
   *   <li>Target speed is above the launch threshold (prevents false triggers)
   *   <li>Virtual hopper has fuel available
   *   <li>Enough time has passed since last launch (rate limiting)
   * </ul>
   *
   * @return true if fuel should be visualized as launching
   */
  private boolean shouldVisualizeLaunch() {
    // Check flywheel is at target speed
    boolean atTargetSpeed = areFlywheelsAtTargetSpeed();

    // Check target speed is above threshold (compare using same units)
    double targetRPM = _targetFlywheelSpeed.in(RevolutionsPerSecond) * 60;
    boolean aboveThreshold = targetRPM > ShooterConstants.SHOOTER_RPM_THRESHOLD_FOR_LAUNCH.in(RPM);

    // Check hopper has fuel
    boolean hasFuel = VirtualHopper.getInstance().hasFuel();

    // Check rate limit
    boolean canLaunch = _fuelSimVisualizer.canLaunch();

    return atTargetSpeed && aboveThreshold && hasFuel && canLaunch;
  }

  /**
   * Launches virtual fuel if all conditions are met.
   *
   * <p>This uses the MEASURED flywheel speed (not target) for realistic physics. The measured speed
   * reflects actual motor performance, giving more accurate trajectory prediction.
   */
  private void visualizeFuelLaunch() {
    if (!shouldVisualizeLaunch()) {
      return;
    }

    // Use MEASURED speed for realistic physics
    LinearVelocity linearSpeed =
        _fuelSimVisualizer.convertToLinearVelocity(_shooterInputs._flywheelMotorVelocity);

    // Launch with hood angle from constants
    _fuelSimVisualizer.launchFuel(linearSpeed, ShooterConstants.SHOOTER_HOOD_ANGLE);
  }

  /**
   * Stops the flywheel by setting target speed to zero.
   *
   * <p>This commands the flywheel to stop spinning. Note that a heavy flywheel takes time to spin
   * down due to its momentum - it won't stop instantly.
   *
   * <p><b>When to use:</b>
   *
   * <ul>
   *   <li>When disabled for safety
   *   <li>After completing shots
   *   <li>When the robot is idle
   * </ul>
   *
   * <p><b>Safety Note:</b> Always wait for the flywheel to fully stop before performing maintenance
   * or reaching near the shooter mechanism.
   */
  public void stopFlywheels() {
    _targetFlywheelSpeed = RadiansPerSecond.of(0);
    _shooterIO.stopFlywheel();
  }

  /**
   * Sets the target flywheel speed.
   *
   * <p>This commands the flywheel to spin at a specific speed. The motor controller's PID will
   * automatically adjust voltage to reach and maintain this speed.
   *
   * <p><b>How to use:</b>
   *
   * <pre>
   * // Set a specific RPM
   * shooter.setFlywheelSpeed(RPM.of(3000));
   *
   * // Use distance-based shooting
   * Distance distanceToHub = drive.getDistanceToHub();
   * shooter.setFlywheelSpeed(shooter.getSpeedForDistance(distanceToHub));
   * </pre>
   *
   * <p>The flywheel will take time to spin up. Use areFlywheelsAtTargetSpeed() to check when it's
   * ready to shoot.
   *
   * @param speed The desired flywheel angular velocity (use RPM.of(), RadiansPerSecond.of(), etc.)
   */
  public void setFlywheelSpeed(AngularVelocity speed) {
    _targetFlywheelSpeed = speed;
    _shooterIO.setFlywheelSpeed(speed);
  }

  /**
   * Calculates the required flywheel speed for a given distance to the target.
   *
   * <p>This is the key method for distance-based shooting. It uses WPILib's
   * InterpolatingDoubleTreeMap which automatically performs linear interpolation between data
   * points.
   *
   * <p><b>How the lookup table works:</b>
   *
   * <ul>
   *   <li>You characterize your shooter by shooting from various distances
   *   <li>Record what RPM is needed for each distance to make the shot
   *   <li>Store these (distance, RPM) pairs in ShooterConstants.DISTANCE_TO_SPEED_MAP
   *   <li>InterpolatingDoubleTreeMap automatically interpolates for distances between your data
   *       points
   * </ul>
   *
   * <p><b>Example lookup table:</b>
   *
   * <pre>
   * Distance (m) | RPM
   * -------------|-----
   * 1.0          | 2000
   * 2.0          | 2500
   * 3.0          | 3200
   * </pre>
   *
   * If you ask for 1.5m, InterpolatingDoubleTreeMap calculates: (2000 + 2500) / 2 = 2250 RPM
   *
   * <p><b>How to characterize your shooter:</b>
   *
   * <ol>
   *   <li>Place robot at known distance from target
   *   <li>Manually adjust RPM until shots consistently score
   *   <li>Record the distance and RPM
   *   <li>Repeat for 5-7 distances across your shooting range
   *   <li>Add data points to ShooterConstants.DISTANCE_TO_SPEED_MAP using the units library
   * </ol>
   *
   * @param distance Distance to the target (use Meters.of() or similar)
   * @return Required flywheel speed as an AngularVelocity
   */
  public AngularVelocity getSpeedForDistance(Distance distance) {
    // Convert distance to meters for lookup table
    double distanceMeters = distance.in(Meters);

    // Get interpolated speed from lookup table (returns RPM)
    double speedRPM = ShooterConstants.DISTANCE_TO_SPEED_MAP.get(distanceMeters);

    // Clamp to min/max speeds for safety
    double minRPM = ShooterConstants.MIN_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0;
    double maxRPM = ShooterConstants.MAX_FLYWHEEL_SPEED.in(RevolutionsPerSecond) * 60.0;
    speedRPM = Math.max(minRPM, Math.min(maxRPM, speedRPM));

    // Log the calculation for debugging
    Logger.recordOutput("Shooter/ShotCalculator/Distance_m", distanceMeters);
    Logger.recordOutput("Shooter/ShotCalculator/CalculatedSpeed_RPM", speedRPM);

    // Convert RPM to AngularVelocity and return
    return RPM.of(speedRPM);
  }

  /**
   * Checks if the flywheel has reached its target speed.
   *
   * <p>This returns true when the flywheel speed is within an acceptable tolerance of the target
   * speed. The tolerance is defined as a percentage in ShooterConstants.
   *
   * <p><b>How it works:</b>
   *
   * <pre>
   * error = |target speed - actual speed|
   * tolerance = target speed × FLYWHEEL_PID_TOLERANCE
   * at target = error <= tolerance
   * </pre>
   *
   * <p><b>Example:</b> If target is 3000 RPM and tolerance is 0.02 (2%):
   *
   * <ul>
   *   <li>Tolerance = 3000 × 0.02 = 60 RPM
   *   <li>At target when speed is between 2940-3060 RPM
   * </ul>
   *
   * <p><b>Usage:</b> Check this before feeding game pieces to the shooter. Feeding before the
   * flywheel is up to speed will result in weak, inaccurate shots.
   *
   * @return true if flywheel is at target speed (within tolerance), false otherwise
   */
  public boolean areFlywheelsAtTargetSpeed() {
    return Math.abs(
            _targetFlywheelSpeed.in(RadiansPerSecond)
                - _shooterInputs._flywheelMotorVelocity.in(RadiansPerSecond))
        <= Math.abs(
            _targetFlywheelSpeed.in(RadiansPerSecond) * ShooterConstants.FLYWHEEL_PID_TOLERANCE);
    // the tolerance is a percent error of the target speed we are allowed
  }

  /**
   * Manually controls the flywheel with a duty cycle output.
   *
   * <p><b>Warning:</b> This bypasses the PID velocity control and directly sets motor power. Use
   * carefully - mainly for testing or manual override.
   *
   * <p>Duty cycle output ranges:
   *
   * <ul>
   *   <li>+1.0 = Full power forward
   *   <li>0.0 = Stopped
   *   <li>-1.0 = Full power reverse (not recommended for shooter!)
   * </ul>
   *
   * <p><b>When to use this:</b>
   *
   * <ul>
   *   <li>Testing motor direction during initial setup
   *   <li>Verifying motor controller wiring
   *   <li>Emergency manual control if PID fails
   * </ul>
   *
   * <p>For normal operation, use setFlywheelSpeed() instead, which provides precise velocity
   * control and consistency.
   *
   * @param output The duty cycle output (0.0 to +1.0 recommended) for the flywheel motor
   */
  public void setFlyWheelDutyCycle(double output) {
    this._shooterIO.setFlyWheelDutyCycle(output);
  }

  public void runCharacterization(Voltage volts) {
    this._shooterIO.setFlywheelVoltage(volts);
  }

  /**
   * Returns the current flywheel velocity used for feedforward characterization.
   *
   * <p>This returns the measured angular velocity in radians per second. The characterization
   * routine uses this value to correlate applied voltage to achieved velocity.
   *
   * @return current flywheel angular velocity (rad/s)
   */
  public double getFFCharacterizationVelocity() {
    return _shooterInputs._flywheelMotorVelocity.in(RadiansPerSecond);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(Volts.of(0.0)))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction));
  }

  /**
   * Updates the flywheel speed to shoot at the alliance hub.
   *
   * <p>This method calculates the required flywheel speed based on the current distance to the
   * alliance hub and sets the flywheel to that speed. Call this method continuously (e.g., from a
   * command's execute()) to adjust speed as the robot moves.
   *
   * <p>The alliance hub target is automatically flipped based on alliance color.
   *
   * <p><b>Usage in a command:</b>
   *
   * <pre>
   * // Continuously update speed while driving
   * Commands.run(() -> shooter.updateSpeedForHub(), shooter)
   * </pre>
   */
  public void updateSpeedForHub() {
    AngularVelocity targetSpeed = ShotCalculator.getInstance().getFlywheelSpeedForAllianceHub();
    setFlywheelSpeed(targetSpeed);
  }

  /**
   * Updates the flywheel speed to shoot at a specific point on the field.
   *
   * <p>This method calculates the required flywheel speed based on the current distance to the
   * target point and sets the flywheel to that speed. Call this method continuously to adjust speed
   * as the robot moves.
   *
   * @param target The target point to shoot at (Translation3d in field coordinates)
   */
  public void updateSpeedForTarget(Translation3d target) {
    AngularVelocity targetSpeed = ShotCalculator.getInstance().getFlywheelSpeedForTarget(target);
    setFlywheelSpeed(targetSpeed);
  }

  /**
   * Checks if the robot is within effective shooting range of the alliance hub.
   *
   * <p>This can be used by commands to decide whether to attempt a shot or provide driver feedback.
   *
   * @return true if the robot is within the characterized shooting range
   */
  public boolean isInShootingRange() {
    return ShotCalculator.getInstance().isInShootingRange();
  }
}
