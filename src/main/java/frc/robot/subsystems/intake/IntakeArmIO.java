package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {


    @AutoLog
    public static class IntakeArmIOInputs{
        // |================= START LEFT FLYWHEEL MOTOR LOGGING =================|
        public double _leftFlywheelMotorTemperature;
        public double _leftFlywheelMotorVelocityRotationsPerMin;
        public double _leftFlywheelMotorVoltage;
        public double _leftFlywheelMotorCurrent;
        // |================= END LEFT FLYWHEEL MOTOR LOGGING =================|

        // |================= START ANGLE MOTOR LOGGING =================|
        public double _angleMotorTemperature;

        public double _angleMotorVelocityRotationsPerMin;
        public double _angleMotorCurrent;
        public double _angleMotorVoltage;
        // |================= END ANGLE MOTOR LOGGING =================|

        // |================= START ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|
        public double _angleEncoderPositionDegrees;
        // |================= END ANGLE DUTY CYCLE ENCODER MOTOR LOGGING =================|

    }


    public default void updateInputs(IntakeArmIOInputs inputs){ }
}
