package frc.robot;

public class HardwareConstants {
  public static final int NUMBER_OF_CAMERAS = 2;

  public class CanIds {
    // |====================== INTAKE SUBSYSTEM CAN IDs ======================|
    public static int RIGHT_PIVOT_MOTOR_ID = 20;
    public static int LEFT_PIVOT_MOTOR_ID = 21;
    public static int ROLLER_MOTOR_ID = 22;

    // |====================== SHOOTER SUBSYSTEM CAN IDs ======================|
    public static int FLYWHEEL_MOTOR_ID = 31;
    public static int FEEDER_MOTOR_ID = 32;

    // |====================== CLIMBER SUBSYSTEM CAN IDs ======================|
    public static int CLIMBER_MOTOR_ID = 42;
  }

  public class DIOPorts {
    public static int INTAKE_UPPER_LIMIT_SWITCH_PORT = 0;
  }
}
