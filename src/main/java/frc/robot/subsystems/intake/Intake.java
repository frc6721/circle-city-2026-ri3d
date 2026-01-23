package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The Intake subsystem controls the robot's game piece intake mechanism.
 *
 * <p><b>Hardware Overview:</b>
 *
 * <ul>
 *   <li>Slap-down intake design with 2 inverted spinning shafts for rollers
 *   <li>Two NEO motors for pivot control (leader-follower configuration)
 *   <li>AM Sport Gearboxes (16:1 reduction) for the roller motors
 *   <li>REV Through Bore Encoder attached to the leader motor for absolute position
 *   <li>Thrifty Squish Wheels (~25) on the roller shafts to grab game pieces
 *   <li>Belt-driven system using 5mm pitch belts and pulleys
 * </ul>
 *
 * <p><b>How It Works:</b>
 *
 * <ul>
 *   <li><b>Pivot:</b> The entire intake pivots up (stow) and down (pickup) using two motors
 *   <li><b>Rollers:</b> When down, the roller shafts spin to pull in game pieces
 *   <li><b>Position Control:</b> A PID controller maintains the pivot at desired angles
 *   <li><b>Absolute Encoder:</b> Through-bore encoder provides accurate position even after power
 *       cycle
 * </ul>
 *
 * <p><b>Software Features:</b>
 *
 * <ul>
 *   <li>Closed-loop (PID) position control for the pivot angle
 *   <li>Feedforward compensation for gravity (cosine of angle)
 *   <li>Predefined positions (STOW and PICKUP) for easy control
 *   <li>Comprehensive logging of position, setpoints, voltages, and PID gains
 * </ul>
 *
 * <p><b>Key Learnings from RI3D:</b>
 *
 * <ul>
 *   <li>The bumper surface was too slick - consider adding a backer or pinch roller
 *   <li>Two-driven-shaft pinch can stall - optimize center-to-center distance
 *   <li>Through-bore encoders provide reliable absolute positioning
 * </ul>
 *
 * <p>This subsystem uses the AdvantageKit IO layer pattern to separate hardware control from the
 * subsystem logic, enabling simulation and easy hardware swaps.
 */
public class Intake extends SubsystemBase {

  private final IntakeIO _intakeIO;
  private final IntakeIOInputsAutoLogged _intakeInputs = new IntakeIOInputsAutoLogged();
  private IntakePosition _intakePosition;
  private PIDController _pivotPIDController;

  /**
   * Enum representing predefined positions for the intake pivot.
   *
   * <p>Using an enum makes it easy to add new positions and ensures we only use valid positions
   * (can't accidentally pass an invalid angle).
   *
   * <p><b>Available Positions:</b>
   *
   * <ul>
   *   <li><b>STOW:</b> Intake is up inside the robot frame, safe for driving
   *   <li><b>PICKUP:</b> Intake is down, ready to collect game pieces from the ground
   * </ul>
   *
   * <p>The actual angles for each position are defined in {@link IntakeConstants} and can be tuned
   * without changing this code.
   */
  public enum IntakePosition {
    STOW(IntakeConstants.POSITION_STOW),
    PICKUP(IntakeConstants.POSITION_PICKUP);

    private final LoggedNetworkNumber _angle;

    /**
     * Private constructor for the IntakePosition enum.
     *
     * <p>This associates each position with its angle value. The angles are stored as
     * LoggedNetworkNumbers so they can be tuned from the dashboard without redeploying code.
     *
     * @param angle The angle for this position (from IntakeConstants)
     */
    private IntakePosition(LoggedNetworkNumber angle) {
      this._angle = angle;
    }

    /**
     * Returns the angle associated with this intake position.
     *
     * <p>The angles are defined in {@link IntakeConstants} and stored as LoggedNetworkNumbers,
     * allowing them to be tuned from the dashboard.
     *
     * <p><b>Example:</b>
     *
     * <pre>
     * IntakePosition.STOW.getAngle() // Returns the stow angle (e.g., 0 degrees)
     * </pre>
     *
     * @return The angle for this position as a Rotation2d
     */
    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(this._angle.get());
    }
  }

  /**
   * Creates a new Intake subsystem.
   *
   * <p>This constructor:
   *
   * <ul>
   *   <li>Stores the hardware IO interface
   *   <li>Initializes the intake position to STOW (assumes intake starts up)
   *   <li>Creates the PID controller with gains from IntakeConstants
   * </ul>
   *
   * <p><b>About the PID Controller:</b> The PID controller automatically adjusts motor voltage to
   * hold the intake at the desired angle:
   *
   * <ul>
   *   <li><b>P (Proportional):</b> Push harder when farther from target
   *   <li><b>I (Integral):</b> Accumulate error over time to overcome steady forces
   *   <li><b>D (Derivative):</b> Slow down when approaching target to prevent overshoot
   * </ul>
   *
   * @param intakeIO The hardware interface for intake control (motors and sensors)
   */
  public Intake(IntakeIO intakeIO) {
    this._intakeIO = intakeIO;

    // assume that the intake is all the way up when first turned on
    _intakePosition = IntakePosition.STOW;
    _pivotPIDController =
        new PIDController(
            IntakeConstants.PIVOT_PID_KP.get(),
            IntakeConstants.PIVOT_PID_KI.get(),
            IntakeConstants.PIVOT_PID_KD.get());
  }

  /**
   * Periodic method called every 20 milliseconds (50 times per second).
   *
   * <p>This method handles:
   *
   * <ul>
   *   <li>Reading sensor data from the intake hardware
   *   <li>Logging all sensor data and setpoints to AdvantageKit
   *   <li>Running the PID control loop to maintain the pivot position
   *   <li>Applying feedforward compensation for gravity
   * </ul>
   *
   * <p><b>PID + Feedforward Control:</b>
   *
   * <ol>
   *   <li>Read current pivot angle from the through-bore encoder
   *   <li>Calculate PID output based on error (desired angle - current angle)
   *   <li>Add feedforward term (cosine of angle) to counteract gravity
   *   <li>Apply total voltage to the pivot motors
   * </ol>
   *
   * <p><b>Why cosine for feedforward?</b> The torque needed to hold the intake depends on the
   * angle:
   *
   * <ul>
   *   <li>At 0° (horizontal): Maximum torque needed (cos(0) = 1)
   *   <li>At 90° (vertical): No torque needed (cos(90) = 0)
   * </ul>
   */
  @Override
  public void periodic() {
    _intakeIO.updateInputs(_intakeInputs);
    Logger.processInputs("Intake", _intakeInputs);

    // LOGGING
    Logger.recordOutput(
        "Intake/Current-Pivot-Angle", _intakeInputs._intakeRightPivotMotorPosition.getDegrees());
    Logger.recordOutput("Intake/Desired-Pivot-Angle", _intakePosition.getAngle().getDegrees());
    Logger.recordOutput(
        "Intake/raw-Pivot-Position",
        _intakeInputs._intakeRightPivotMotorPosition.plus(IntakeConstants.PIVOT_ZERO_ROTATION));
    Logger.recordOutput(
        "Intake/raw-pivot-position-desired",
        _intakePosition.getAngle().plus(IntakeConstants.PIVOT_ZERO_ROTATION));
    _pivotPIDController.setSetpoint(_intakePosition.getAngle().getDegrees());
    double pivotVoltage =
        _pivotPIDController.calculate(_intakeInputs._intakeRightPivotMotorPosition.getDegrees());
    _intakeIO.setPivotMotorVoltage(
        pivotVoltage
            + IntakeConstants.INTAKE_PIVOT_FEEDFORWARD
                * Math.cos(Math.toRadians(_intakePosition.getAngle().getDegrees())));
  }

  /**
   * Sets the intake pivot to a desired position.
   *
   * <p>This method changes the target position for the PID controller. The actual movement happens
   * in the periodic() method.
   *
   * <p><b>Usage in Commands:</b>
   *
   * <pre>
   * // Move intake to pickup position
   * intake.setIntakePosition(IntakePosition.PICKUP);
   * </pre>
   *
   * @param position The desired intake position (STOW or PICKUP)
   */
  public void setIntakePosition(IntakePosition position) {
    _intakePosition = position;
  }

  /**
   * Manually controls the intake pivot with a duty cycle output.
   *
   * <p><b>Warning:</b> This bypasses the PID controller and directly controls the motor. Use
   * carefully - mainly for testing or manual override.
   *
   * <p>Duty cycle output ranges:
   *
   * <ul>
   *   <li>+1.0 = Full power in one direction
   *   <li>0.0 = Stopped
   *   <li>-1.0 = Full power in opposite direction
   * </ul>
   *
   * <p><b>When to use this:</b> Testing motor directions, manual control during setup, or emergency
   * override. For normal operation, use setIntakePosition() instead.
   *
   * @param output The duty cycle output (-1.0 to +1.0) for the pivot motor
   */
  public void setIntakePivotDutyCucleOutput(double output) {
    _intakeIO.setIntakePivotDutyCucleOutput(output);
  }

  /**
   * Turns on the intake roller motors to acquire game pieces.
   *
   * <p>This spins the two roller shafts with Thrifty Squish Wheels to pull game pieces into the
   * robot. The speed is set by INTAKE_ACQUIRE_SPEED in IntakeConstants.
   *
   * <p><b>Usage:</b> Call this when the intake is in PICKUP position and you want to collect a game
   * piece.
   *
   * <p><b>Important:</b> Remember to call stopRollers() when done to save battery and prevent
   * accidentally ejecting the game piece.
   */
  public void turnOnIntakeRollers() {
    _intakeIO.setRollerMotorOutput(IntakeConstants.INTAKE_ACQUIRE_SPEED.get());
  }

  /**
   * Stops the intake roller motors.
   *
   * <p>This sets the roller motor output to 0, stopping the wheels from spinning.
   *
   * <p><b>When to use:</b>
   *
   * <ul>
   *   <li>After successfully acquiring a game piece
   *   <li>When the intake is being stowed
   *   <li>During teleop when the driver releases the intake button
   * </ul>
   */
  public void stopRollers() {
    _intakeIO.setRollerMotorOutput(0.0);
  }
}
