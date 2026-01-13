package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
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
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            IntakeConstants.PIVOT_PID_MIN_INPUT, IntakeConstants.PIVOT_PID_MAX_INPUT)
        .pidf(
            IntakeConstants.PIVOT_PID_KP.get(),
            IntakeConstants.PIVOT_PID_KI.get(),
            IntakeConstants.PIVOT_PID_KD.get(),
            0);

    // Soft limits based on absolute position
    rightConfig.softLimit.forwardSoftLimitEnabled(true);
    rightConfig.softLimit.forwardSoftLimit(IntakeConstants.MAX_INTAKE_ANGLE);
    rightConfig.softLimit.reverseSoftLimitEnabled(true);
    rightConfig.softLimit.reverseSoftLimit(IntakeConstants.MIN_INTAKE_ANGLE);

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
        .inverted(IntakeConstants.INTAKE_LEFT_PIVOT_INVERTED)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.INTAKE_PIVOT_SMART_CURRENT_LIMIT)
        .secondaryCurrentLimit(IntakeConstants.INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT)
        .voltageCompensation(12.0);

    // Configure left motor to follow right motor
    leftConfig.follow(_rightPivotMotor);

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
    config
        .encoder
        .positionConversionFactor(
            2
                * Math.PI
                / IntakeConstants.INTAKE_ROLLER_CONVERSION_FACTOR) // motor rotations to intake rad
        .velocityConversionFactor(
            (2 * Math.PI)
                / 60.0
                / IntakeConstants.INTAKE_ROLLER_CONVERSION_FACTOR) // motor RPM to intake rad/s
        .inverted(false);

    tryUntilOk(
        _rollerMotor,
        5,
        () ->
            _rollerMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    // |================= START RIGHT INTAKE PIVOT MOTOR LOGGING =================|
    inputs._intakeRightPivotMotorTemperature = Celsius.of(_rightPivotMotor.getMotorTemperature());
    inputs._intakeRightPivotMotorVelocity = RadiansPerSecond.of(_pivotEncoder.getVelocity());
    inputs._intakeRightPivotMotorPosition =
        new Rotation2d(_pivotEncoder.getPosition()).minus(IntakeConstants.PIVOT_ZERO_ROTATION);
    inputs._intakeRightPivotMotorVoltage =
        Volts.of(_rightPivotMotor.getAppliedOutput() * _rightPivotMotor.getBusVoltage());
    inputs._intakeRightPivotMotorCurrent = Amps.of(_rightPivotMotor.getOutputCurrent());
    // |================= END RIGHT INTAKE PIVOT MOTOR LOGGING =================|

    // |================= START LEFT INTAKE PIVOT MOTOR LOGGING =================|
    inputs._intakeLeftPivotMotorTemperature = Celsius.of(_leftPivotMotor.getMotorTemperature());
    inputs._intakeLeftPivotMotorVelocity =
        RadiansPerSecond.of(_leftPivotMotor.getEncoder().getVelocity());
    inputs._intakeLeftPivotMotorPosition =
        new Rotation2d(_leftPivotMotor.getEncoder().getPosition());
    inputs._intakeLeftPivotMotorVoltage =
        Volts.of(_leftPivotMotor.getAppliedOutput() * _leftPivotMotor.getBusVoltage());
    inputs._intakeLeftPivotMotorCurrent = Amps.of(_leftPivotMotor.getOutputCurrent());
    // |================= END LEFT INTAKE PIVOT MOTOR LOGGING =================|

    // |================= START INTAKE ROLLER MOTOR LOGGING =================|
    inputs._intakeRollerMotorTemperature = Celsius.of(_rollerMotor.getMotorTemperature());
    inputs._intakeRollerMotorVelocity =
        RadiansPerSecond.of(_rollerMotor.getEncoder().getVelocity());
    inputs._intakeRollerMotorVoltage =
        Volts.of(_rollerMotor.getAppliedOutput() * _rollerMotor.getBusVoltage());
    inputs._intakeRollerMotorCurrent = Amps.of(_rollerMotor.getOutputCurrent());
    // |================= END INTAKE ROLLER MOTOR LOGGING =================|
  }

  // |============================== PIVOT MOTOR METHODS ============================== |
  public void setPivotTargetPosition(Rotation2d angle) {
    _rightPivotMotor
        .getClosedLoopController()
        .setReference(
            angle.plus(IntakeConstants.PIVOT_ZERO_ROTATION).getRadians(), ControlType.kPosition);
  }

  // |============================== ROLLER MOTOR METHODS ============================== |
  public void setRollerMotorOutput(double output) {
    _rollerMotor.set(output);
  }
}
