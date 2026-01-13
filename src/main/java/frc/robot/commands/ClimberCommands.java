package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimberCommands {

  private static final double DEADBAND = 0.1;

  public ClimberCommands() {}

  /**
   * Command to control the climber with a joystick
   *
   * @param climber The climber subsystem
   * @param speedSupplier Supplier providing joystick value (-1.0 to 1.0)
   * @return Command that runs the climber based on joystick input
   */
  public static Command joystickControl(Climber climber, DoubleSupplier speedSupplier) {
    return Commands.run(
            () -> {
              // Get joystick value and apply deadband
              double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);

              // Set climber speed
              climber.setClimberSpeed(speed);
            },
            climber)
        .finallyDo(() -> climber.stopClimber()); // Stop motor when command ends
  }

  /**
   * Command to stop the climber
   *
   * @param climber The climber subsystem
   * @return Command that stops the climber
   */
  public static Command stopClimber(Climber climber) {
    return Commands.runOnce(() -> climber.stopClimber(), climber);
  }
}
