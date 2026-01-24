package frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeConstants;

/**
 * Simulation implementation of IntakeIO that uses physics simulation for realistic behavior.
 *
 * <p>This class simulates:
 *
 * <ul>
 *   <li><b>Pivot:</b> Uses WPILib's SingleJointedArmSim for realistic arm physics with gravity
 *   <li><b>Roller:</b> Simple voltage/current simulation without physics (instant response)
 *   <li><b>Motors:</b> Uses REV SparkMaxSim for realistic motor behavior
 * </ul>
 *
 * <p>The simulation includes:
 *
 * <ul>
 *   <li>Gravity effects on the arm
 *   <li>Moment of inertia and arm length from IntakeConstants
 *   <li>Min/max angle limits (hard stops)
 *   <li>Battery voltage simulation
 * </ul>
 */
public class SimIntakeIO implements IntakeIO {

  // Simulated motors
  private final SparkMax rightPivotMotor;
  private final SparkMax leftPivotMotor;
  private final SparkMax rollerMotor;

  // REV simulation wrappers
  private final SparkMaxSim rightPivotSim;
  private final SparkMaxSim leftPivotSim;
  private final SparkMaxSim rollerSim;

  // WPILib physics simulation for the pivot arm
  private final SingleJointedArmSim pivotArmSim;

  // Simulated roller state
  private double rollerDutyCycle = 0.0;
  private double rollerSimulatedCurrent = 0.0;

  // Pivot control state
  private double pivotAppliedVoltage = 0.0;

  /**
   * Creates a new SimIntakeIO with physics simulation.
   *
   * <p>Initializes simulated motors and the arm physics simulation using constants from
   * IntakeConstants.
   */
  public SimIntakeIO() {
    // Create simulated motors (using arbitrary CAN IDs since they don't matter in sim)
    rightPivotMotor = new SparkMax(50, MotorType.kBrushless);
    leftPivotMotor = new SparkMax(51, MotorType.kBrushless);
    rollerMotor = new SparkMax(52, MotorType.kBrushless);

    // Configure right pivot motor
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .inverted(IntakeConstants.INTAKE_RIGHT_PIVOT_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_PIVOT_SMART_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    rightPivotMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure left pivot motor to follow right
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake).follow(rightPivotMotor, true);

    leftPivotMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure roller motor
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig
        .inverted(IntakeConstants.INTAKE_ROLLER_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_ROLLER_SMART_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create REV simulation wrappers
    rightPivotSim = new SparkMaxSim(rightPivotMotor, IntakeConstants.PIVOT_MOTOR);
    leftPivotSim = new SparkMaxSim(leftPivotMotor, IntakeConstants.PIVOT_MOTOR);
    rollerSim = new SparkMaxSim(rollerMotor, IntakeConstants.ROLLER_MOTOR);

    // Create the arm physics simulation
    // Note: SingleJointedArmSim expects angles in radians
    pivotArmSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                IntakeConstants.PIVOT_MOTOR,
                IntakeConstants.PIVOT_MOI.in(KilogramSquareMeters),
                IntakeConstants.PIVOT_GEAR_RATIO),
            IntakeConstants.PIVOT_MOTOR,
            IntakeConstants.PIVOT_GEAR_RATIO,
            IntakeConstants.ARM_LENGTH.in(Meters),
            Math.toRadians(IntakeConstants.MIN_INTAKE_ANGLE_DEGREES),
            Math.toRadians(IntakeConstants.MAX_INTAKE_ANGLE_DEGREES),
            true, // Simulate gravity
            Math.toRadians(IntakeConstants.STARTING_ANGLE_DEGREES));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // --- Update the arm physics simulation ---

    // Set the input voltage to the simulation
    pivotArmSim.setInputVoltage(pivotAppliedVoltage);

    // Step the simulation forward (20ms = 0.02s)
    pivotArmSim.update(0.02);

    // Update battery simulation based on current draw
    double totalCurrent = pivotArmSim.getCurrentDrawAmps() + Math.abs(rollerSimulatedCurrent);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent));

    // Update the SparkMax simulation with the physics results
    rightPivotSim.setBusVoltage(RoboRioSim.getVInVoltage());
    leftPivotSim.setBusVoltage(RoboRioSim.getVInVoltage());
    rollerSim.setBusVoltage(RoboRioSim.getVInVoltage());

    // Iterate the motor simulations
    rightPivotSim.iterate(
        Math.toDegrees(pivotArmSim.getVelocityRadPerSec()) / 360.0 * 60.0, // Convert to RPM
        RoboRioSim.getVInVoltage(),
        0.02);

    leftPivotSim.iterate(
        Math.toDegrees(pivotArmSim.getVelocityRadPerSec()) / 360.0 * 60.0,
        RoboRioSim.getVInVoltage(),
        0.02);

    // --- Read back the simulated sensor values ---

    // Right pivot motor inputs
    inputs._intakeRightPivotMotorTemperature = Celsius.of(40.0); // Simulated constant temp
    inputs._intakeRightPivotMotorVelocity = RadiansPerSecond.of(pivotArmSim.getVelocityRadPerSec());

    // Transform from physics frame (0° = horizontal) to robot frame (0° = vertical)
    // Robot frame: 0° = vertical (stowed), 90° = horizontal (deployed)
    // Physics frame: 0° = horizontal, 90° = vertical
    // Transformation: robot_angle = 90° - physics_angle
    Rotation2d simAngle = Rotation2d.fromRadians(Math.PI / 2 - pivotArmSim.getAngleRads());
    inputs._intakeRightPivotMotorPosition = simAngle;

    inputs._intakeRightPivotMotorVoltage = Volts.of(pivotAppliedVoltage);
    inputs._intakeRightPivotMotorCurrent = Amps.of(pivotArmSim.getCurrentDrawAmps() / 2.0);

    // Left pivot motor inputs (follower, similar values)
    inputs._intakeLeftPivotMotorTemperature = Celsius.of(40.0);
    inputs._intakeLeftPivotMotorVelocity = RadiansPerSecond.of(pivotArmSim.getVelocityRadPerSec());
    inputs._intakeLeftPivotMotorPosition = simAngle;
    inputs._intakeLeftPivotMotorVoltage = Volts.of(pivotAppliedVoltage);
    inputs._intakeLeftPivotMotorCurrent = Amps.of(pivotArmSim.getCurrentDrawAmps() / 2.0);

    // Roller motor inputs (simple simulation)
    inputs._intakeRollerMotorTemperature = Celsius.of(35.0);
    inputs._intakeRollerMotorVelocity =
        RadiansPerSecond.of(rollerDutyCycle * 500.0); // Approximate velocity
    inputs._intakeRollerMotorVoltage = Volts.of(rollerDutyCycle * 12.0);
    rollerSimulatedCurrent = Math.abs(rollerDutyCycle) * 20.0; // Approximate current draw
    inputs._intakeRollerMotorCurrent = Amps.of(rollerSimulatedCurrent);
  }

  @Override
  public void setPivotMotorVoltage(double volts) {
    // Clamp voltage to battery voltage
    pivotAppliedVoltage = -Math.max(-12.0, Math.min(12.0, volts));
    rightPivotMotor.setVoltage(pivotAppliedVoltage);
  }

  @Override
  public void setIntakePivotDutyCucleOutput(double output) {
    // Convert duty cycle to voltage
    pivotAppliedVoltage = -output * 12.0;
    rightPivotMotor.set(output);
  }

  @Override
  public void setRollerMotorOutput(double output) {
    rollerDutyCycle = Math.max(-1.0, Math.min(1.0, output));
    rollerMotor.set(rollerDutyCycle);
  }
}
