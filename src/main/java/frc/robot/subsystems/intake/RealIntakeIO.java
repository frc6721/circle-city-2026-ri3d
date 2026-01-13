package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class RealIntakeIO implements IntakeIO {
  private SparkMax _rightPivotMotor;
  private SparkMax _leftPivotMotor;
  private SparkMax _rollerMotor;
  private AbsoluteEncoder _pivotEncoder;

  public RealIntakeIO() {
    configPivotMotors();
    configRollerMotor();
  }

  public void configPivotMotors() {
    // Configure right (leader) pivot motor
    _rightPivotMotor =
        new SparkMax(HardwareConstants.CanIds.RIGHT_PIVOT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .inverted(IntakeConstants.INTAKE_RIGHT_PIVOT_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_PIVOT_SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(IntakeConstants.INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    // Configure absolute encoder on right motor
    rightConfig
        .absoluteEncoder
        .inverted(IntakeConstants.PIVOT_ENCODER_INVERTED)
        .positionConversionFactor(IntakeConstants.PIVOT_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(IntakeConstants.PIVOT_ENCODER_VELOCITY_FACTOR)
        .averageDepth(2);

    // Configure closed loop to use absolute encoder
    rightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(false) // Changed: Don't wrap for a limited-range mechanism
        .pidf(
            IntakeConstants.PIVOT_PID_KP.get(),
            IntakeConstants.PIVOT_PID_KI.get(),
            IntakeConstants.PIVOT_PID_KD.get(),
            0);

    // Soft limits based on absolute position (accounting for zero offset)
    // double minLimitRad =
    //     Degree.of(IntakeConstants.MIN_INTAKE_ANGLE).in(Radians)
    //         + IntakeConstants.PIVOT_ZERO_ROTATION.getRadians();
    // double maxLimitRad =
    //     Degree.of(IntakeConstants.MAX_INTAKE_ANGLE).in(Radians)
    //         + IntakeConstants.PIVOT_ZERO_ROTATION.getRadians();

    // rightConfig.softLimit.forwardSoftLimitEnabled(true);
    // rightConfig.softLimit.forwardSoftLimit(maxLimitRad);
    // rightConfig.softLimit.reverseSoftLimitEnabled(true);
    // rightConfig.softLimit.reverseSoftLimit(minLimitRad);

    tryUntilOk(
        _rightPivotMotor,
        5,
        () ->
            _rightPivotMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    _pivotEncoder = _rightPivotMotor.getAbsoluteEncoder();

    // Configure left (follower) pivot motor
    _leftPivotMotor =
        new SparkMax(HardwareConstants.CanIds.LEFT_PIVOT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_PIVOT_SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(IntakeConstants.INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    // Configure left motor to follow right motor
    leftConfig.follow(_rightPivotMotor, true);

    tryUntilOk(
        _leftPivotMotor,
        5,
        () ->
            _leftPivotMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void configRollerMotor() {
    _rollerMotor = new SparkMax(HardwareConstants.CanIds.ROLLER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(IntakeConstants.INTAKE_ROLLER_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_ROLLER_SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(IntakeConstants.INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    tryUntilOk(
        _rollerMotor,
        5,
        () ->
            _rollerMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    // |================= START RIGHT INTAKE PIVOT MOTOR LOGGING =================|
    ifOk(
        _rightPivotMotor,
        _rightPivotMotor::getMotorTemperature,
        (value) -> inputs._intakeRightPivotMotorTemperature = Celsius.of(value));
    ifOk(
        _rightPivotMotor,
        _pivotEncoder::getVelocity,
        (value) -> inputs._intakeRightPivotMotorVelocity = RadiansPerSecond.of(value));
    ifOk(
        _rightPivotMotor,
        _pivotEncoder::getPosition,
        (value) ->
            inputs._intakeRightPivotMotorPosition =
                new Rotation2d(value).minus(IntakeConstants.PIVOT_ZERO_ROTATION));
    ifOk(
        _rightPivotMotor,
        new java.util.function.DoubleSupplier[] {
          _rightPivotMotor::getAppliedOutput, _rightPivotMotor::getBusVoltage
        },
        (values) -> inputs._intakeRightPivotMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _rightPivotMotor,
        _rightPivotMotor::getOutputCurrent,
        (value) -> inputs._intakeRightPivotMotorCurrent = Amps.of(value));
    // |================= END RIGHT INTAKE PIVOT MOTOR LOGGING =================|

    // |================= START LEFT INTAKE PIVOT MOTOR LOGGING =================|
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor::getMotorTemperature,
        (value) -> inputs._intakeLeftPivotMotorTemperature = Celsius.of(value));
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor.getEncoder()::getVelocity,
        (value) -> inputs._intakeLeftPivotMotorVelocity = RadiansPerSecond.of(value));
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor.getEncoder()::getPosition,
        (value) -> inputs._intakeLeftPivotMotorPosition = new Rotation2d(value));
    ifOk(
        _leftPivotMotor,
        new java.util.function.DoubleSupplier[] {
          _leftPivotMotor::getAppliedOutput, _leftPivotMotor::getBusVoltage
        },
        (values) -> inputs._intakeLeftPivotMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _leftPivotMotor,
        _leftPivotMotor::getOutputCurrent,
        (value) -> inputs._intakeLeftPivotMotorCurrent = Amps.of(value));
    // |================= END LEFT INTAKE PIVOT MOTOR LOGGING =================|

    // |================= START INTAKE ROLLER MOTOR LOGGING =================|
    ifOk(
        _rollerMotor,
        _rollerMotor::getMotorTemperature,
        (value) -> inputs._intakeRollerMotorTemperature = Celsius.of(value));
    ifOk(
        _rollerMotor,
        _rollerMotor.getEncoder()::getVelocity,
        (value) -> inputs._intakeRollerMotorVelocity = RadiansPerSecond.of(value));
    ifOk(
        _rollerMotor,
        new java.util.function.DoubleSupplier[] {
          _rollerMotor::getAppliedOutput, _rollerMotor::getBusVoltage
        },
        (values) -> inputs._intakeRollerMotorVoltage = Volts.of(values[0] * values[1]));
    ifOk(
        _rollerMotor,
        _rollerMotor::getOutputCurrent,
        (value) -> inputs._intakeRollerMotorCurrent = Amps.of(value));
    // |================= END INTAKE ROLLER MOTOR LOGGING =================|
  }

  // |============================== PIVOT MOTOR METHODS ============================== |
  public void setPivotTargetPosition(Rotation2d angle) {
    Logger.recordOutput("Intake/pivot-target-angle", angle.getDegrees());
    // Add zero offset to convert from mechanism angle to encoder angle
    double targetRad = angle.plus(IntakeConstants.PIVOT_ZERO_ROTATION).getRadians();
    Logger.recordOutput("Intake/pivot-target-angle-encoder-space", Math.toDegrees(targetRad));
    _rightPivotMotor.getClosedLoopController().setReference(targetRad, ControlType.kPosition);
  }

  public void setIntakePivotDutyCucleOutput(double output) {
    _rightPivotMotor.set(output);
  }

  // |============================== ROLLER MOTOR METHODS ============================== |
  public void setRollerMotorOutput(double output) {
    _rollerMotor.set(output);
  }
}
