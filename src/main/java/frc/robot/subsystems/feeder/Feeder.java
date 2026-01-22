package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The Feeder subsystem controls the mechanism that feeds game pieces from the hopper
 * into the shooter.
 *
 * <p><b>Hardware Overview:</b>
 * <ul>
 *   <li>Single NEO motor with AM Sport Gearbox (4:1 reduction)</li>
 *   <li>4" Thrifty Squish Wheels to grip and move game pieces</li>
 *   <li>Belt-driven system using 36T pulleys and 120T 5mm pitch belt</li>
 *   <li>Mounted to the shooter uprights using AM Sport brackets</li>
 *   <li>Polycarbonate arc guides game pieces from hopper to shooter</li>
 * </ul>
 *
 * <p><b>How It Works:</b>
 * <ul>
 *   <li><b>Indexing:</b> Holds game pieces in position until ready to shoot</li>
 *   <li><b>Feeding:</b> Pushes game pieces into the spinning flywheel</li>
 *   <li><b>Simple Control:</b> Just runs forward, backward, or stops - no complex control needed</li>
 * </ul>
 *
 * <p><b>Software Design Philosophy:</b>
 * This subsystem is intentionally simple. Unlike the shooter (PID velocity control) or
 * intake (PID position control), the feeder just needs to:
 * <ol>
 *   <li>Wait for the shooter to reach target RPM</li>
 *   <li>Turn on to feed the game piece</li>
 *   <li>Turn off after the shot</li>
 * </ol>
 *
 * <p><b>Integration with Shooter:</b>
 * The feeder is typically controlled by commands that coordinate it with the shooter:
 * <ul>
 *   <li>Shooter spins up to target RPM</li>
 *   <li>Once at speed, feeder automatically activates</li>
 *   <li>Game piece launches</li>
 *   <li>Feeder stops</li>
 * </ul>
 *
 * <p><b>Key Features:</b>
 * <ul>
 *   <li>Duty cycle control (simple -1.0 to +1.0 speed control)</li>
 *   <li>Logs motor voltage and current for diagnostics</li>
 *   <li>No sensors needed - operates open-loop</li>
 * </ul>
 *
 * <p>This subsystem uses the AdvantageKit IO layer pattern to separate hardware control
 * from the subsystem logic.
 */
public class Feeder extends SubsystemBase {

  private final FeederIO _feederIO;
  private final FeederIOInputsAutoLogged _feederInputs = new FeederIOInputsAutoLogged();

  /**
   * Creates a new Feeder subsystem.
   *
   * <p>Initializes the feeder with the provided hardware IO interface.
   *
   * <p>The IO layer (FeederIO) handles all hardware-specific details like motor
   * configuration, CAN IDs, and inversions. This subsystem just provides the
   * high-level interface for controlling the feeder.
   *
   * @param feederIO The hardware interface for feeder control (motor and sensors)
   */
  public Feeder(FeederIO feederIO) {
    this._feederIO = feederIO;
  }

  /**
   * Periodic method called every 20 milliseconds (50 times per second).
   *
   * <p>This method:
   * <ul>
   *   <li>Reads sensor data from the feeder hardware (motor voltage, current, etc.)</li>
   *   <li>Logs all data to AdvantageKit for analysis and debugging</li>
   * </ul>
   *
   * <p>Unlike subsystems with closed-loop control, the feeder doesn't need to
   * calculate anything in periodic - it just reads and logs. The motor speed is
   * set directly by commands calling setFeederSpeed().
   */
  @Override
  public void periodic() {
    // Update and log sensor inputs from hardware
    _feederIO.updateInputs(_feederInputs);
    Logger.processInputs("Feeder", _feederInputs);
  }

  /**
   * Sets the feeder motor speed.
   *
   * <p>This directly controls the motor using duty cycle (percentage power).
   * No PID or feedback control - just open-loop speed control.
   *
   * <p><b>Speed Values:</b>
   * <ul>
   *   <li>+1.0 = Full power forward (feed into shooter)</li>
   *   <li>+0.5 = Half power forward</li>
   *   <li>0.0 = Stopped</li>
   *   <li>-0.5 = Half power reverse</li>
   *   <li>-1.0 = Full power reverse (pull back from shooter)</li>
   * </ul>
   *
   * <p><b>Typical usage:</b>
   * <pre>
   * // Feed game piece at full speed
   * feeder.setFeederSpeed(1.0);
   *
   * // Feed at 70% speed (gentler)
   * feeder.setFeederSpeed(0.7);
   *
   * // Reverse to clear jam
   * feeder.setFeederSpeed(-0.3);
   * </pre>
   *
   * <p><b>Best Practices:</b>
   * <ul>
   *   <li>Only feed when shooter is at target RPM for consistent shots</li>
   *   <li>Use a constant from FeederConstants for standard feed speed</li>
   *   <li>Remember to call stop() when done.</li>
   * </ul>
   *
   * @param speed Duty cycle from -1.0 (full reverse) to +1.0 (full forward)
   */
  public void setFeederSpeed(double speed) {
    _feederIO.setMotorSpeed(speed);
  }

  /**
   * Stops the feeder motor.
   *
   * <p>This is a convenience method that sets the motor speed to 0.
   * Equivalent to calling setFeederSpeed(0.0).
   *
   */
  public void stop() {
    _feederIO.setMotorSpeed(0.0);
  }
}
