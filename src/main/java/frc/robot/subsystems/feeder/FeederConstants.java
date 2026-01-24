package frc.robot.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.Logger;

public class FeederConstants {

  // ==================== MECHANICAL CONSTANTS ====================

  /** Gear ratio from motor to feeder mechanism (motor rotations per mechanism rotation) */
  public static final double FEEDER_GEAR_RATIO = 4.0; // 4:1 AM Sport Gearbox

  /** Motor type for the feeder (1x NEO) */
  public static final DCMotor FEEDER_MOTOR = DCMotor.getNEO(1);

  // ==================== MOTOR CONFIGURATION ====================

  /** Motor inversion */
  public static final boolean FEEDER_MOTOR_INVERTED = false;

  // ==================== CURRENT LIMITS ====================

  public static final int FEEDER_MOTOR_SMART_CURRENT_LIMIT = 40;
  public static final double FEEDER_MOTOR_SECONDARY_CURRENT_LIMIT = 55;

  // ==================== LOGGING ====================

  static {
    Logger.recordOutput("Constants/Feeder/FEEDER_GEAR_RATIO", FEEDER_GEAR_RATIO);
    Logger.recordOutput(
        "Constants/Feeder/FEEDER_MOTOR_SMART_CURRENT_LIMIT", FEEDER_MOTOR_SMART_CURRENT_LIMIT);
    Logger.recordOutput(
        "Constants/Feeder/FEEDER_MOTOR_SECONDARY_CURRENT_LIMIT",
        FEEDER_MOTOR_SECONDARY_CURRENT_LIMIT);
    Logger.recordOutput("Constants/Feeder/FEEDER_MOTOR_INVERTED", FEEDER_MOTOR_INVERTED);
  }
}
