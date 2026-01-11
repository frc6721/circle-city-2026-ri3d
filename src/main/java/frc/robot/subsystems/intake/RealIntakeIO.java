package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.HardwareConstants;
import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;

public class RealIntakeIO implements IntakeIO {
    private SparkMax _pivotMotor;
    private SparkMax _rollerMotor;
    private DigitalInput _upperLimitSwitch;

    public RealIntakeIO() {
        configPivotMotor();
        configRollerMotor();
        _upperLimitSwitch = new DigitalInput(HardwareConstants.DIOPorts.INTAKE_UPPER_LIMIT_SWITCH_PORT);
    }

    public void configPivotMotor(){
        _pivotMotor = new SparkMax(HardwareConstants.CanIds.PIVOT_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(IntakeConstants.INTAKE_PIVOT_INVERTED)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(IntakeConstants.INTAKE_PIVOT_SMART_CURRENT_LIMIT)
              .secondaryCurrentLimit(IntakeConstants.INTAKE_PIVOT_SECONDARY_CURRENT_LIMIT)
              .voltageCompensation(12.0);
        
        config.encoder
            .positionConversionFactor(2 * Math.PI / IntakeConstants.INTAKE_PIVOT_CONVERSION_FACTOR) // motor rotations to intake rad
            .velocityConversionFactor((2 * Math.PI) / 60.0 / IntakeConstants.INTAKE_PIVOT_CONVERSION_FACTOR)// motor RPM to intake rad/s
            .inverted(false);
        config.closedLoop
            .pidf(
                    IntakeConstants.PIVOT_PID_KP.get(), 
                    IntakeConstants.PIVOT_PID_KI.get(), 
                    IntakeConstants.PIVOT_PID_KD.get(), 
                    0
                );

        tryUntilOk(
            _pivotMotor,
            5,
            () ->
            _pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void configRollerMotor(){
        _rollerMotor = new SparkMax(HardwareConstants.CanIds.ROLLER_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(IntakeConstants.INTAKE_ROLLER_INVERTED)
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(IntakeConstants.INTAKE_ROLLER_SMART_CURRENT_LIMIT)
              .secondaryCurrentLimit(IntakeConstants.INTAKE_ROLLER_SECONDARY_CURRENT_LIMIT)
              .voltageCompensation(12.0);
        
        config.encoder
            .positionConversionFactor(2 * Math.PI / IntakeConstants.INTAKE_ROLLER_CONVERSION_FACTOR) // motor rotations to intake rad
            .velocityConversionFactor((2 * Math.PI) / 60.0 / IntakeConstants.INTAKE_ROLLER_CONVERSION_FACTOR)// motor RPM to intake rad/s
            .inverted(false);

        tryUntilOk(
            _rollerMotor,
            5,
            () ->
            _rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }
    
    public void updateInputs(IntakeArmIOInputs inputs){
        // |================= START INTAKE PIVOT MOTOR LOGGING =================|
        inputs._intakePivotMotorTemperature = Celsius.of(_pivotMotor.getMotorTemperature());
        inputs._intakePivotMotorVelocity = RadiansPerSecond.of(_pivotMotor.getEncoder().getVelocity());
        inputs._intakePivotMotorPosition = Radians.of(_pivotMotor.getEncoder().getPosition());
        inputs._intakePivotMotorVoltage = Volts.of(_pivotMotor.getAppliedOutput() * _pivotMotor.getBusVoltage());
        inputs._intakePivotMotorCurrent = Amps.of(_pivotMotor.getOutputCurrent());
        // |================= END INTAKE PIVOT MOTOR LOGGING =================|

        // |================= START INTAKE ROLLER MOTOR LOGGING =================|
        inputs._intakeRollerMotorTemperature = Celsius.of(_rollerMotor.getMotorTemperature());
        inputs._intakeRollerMotorVelocity = RadiansPerSecond.of(_rollerMotor.getEncoder().getVelocity());
        inputs._intakeRollerMotorPosition = Radians.of(_rollerMotor.getEncoder().getPosition());
        inputs._intakeRollerMotorVoltage = Volts.of(_rollerMotor.getAppliedOutput() * _rollerMotor.getBusVoltage());
        inputs._intakeRollerMotorCurrent = Amps.of(_rollerMotor.getOutputCurrent());
        // |================= END INTAKE ROLLER MOTOR LOGGING =================|

        // |================= START LIMIT SWITCH LOGGING =================|
        inputs._upperLimitSwitchTriggered = _upperLimitSwitch.get();
        // |================= END LIMIT SWITCH LOGGING =================|
     }

    // |============================== PIVOT MOTOR METHODS ============================== |
    public void setPivotTargetPosition(Angle angle) {
        _pivotMotor.getClosedLoopController().setReference(angle.in(Radians), ControlType.kPosition);
    }
    public void setPivotMotorSpeed(AngularVelocity speed) {
        
    }

    // |============================== ROLLER MOTOR METHODS ============================== |  
    public void setRollerMotorSpeed(AngularVelocity speed) {}
}
