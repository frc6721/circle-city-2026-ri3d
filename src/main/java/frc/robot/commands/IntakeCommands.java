package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

/**
 * Factory class for creating intake-related commands.
 *
 * <p>This class provides static factory methods that create commands for common intake operations.
 * Using factory methods keeps command creation logic in one place and makes RobotContainer cleaner.
 *
 * <p><b>Why use factory methods instead of custom Command classes?</b>
 *
 * <ul>
 *   <li>Simpler - no need to create separate class files for simple actions
 *   <li>Less boilerplate - Commands.run() handles the command framework
 *   <li>Easier to read - all intake commands in one file
 *   <li>Still composable - can be combined with other commands
 * </ul>
 *
 * <p><b>Usage in RobotContainer:</b>
 *
 * <pre>
 * // Bind to controller buttons
 * driver.a().onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.PICKUP));
 * driver.b().onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.STOW));
 * driver.leftBumper().whileTrue(IntakeCommands.runIntakeRollers(intake));
 * </pre>
 */
public class IntakeCommands {

  /**
   * Creates a command to set the intake pivot to a target position.
   *
   * <p>This command sets the intake's target position once and immediately finishes. The PID
   * controller in the Intake subsystem's periodic() method continues to handle the actual movement
   * to reach and maintain the position even after this command ends.
   *
   * <p><b>How it works:</b>
   *
   * <ol>
   *   <li>Command calls intake.setIntakePosition() once
   *   <li>Intake subsystem updates its target position
   *   <li>Command finishes immediately
   *   <li>PID controller (running in periodic()) drives motors to reach target
   * </ol>
   *
   * <p><b>Usage examples:</b>
   *
   * <pre>
   * // Deploy intake when button pressed
   * driver.a().onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.PICKUP));
   *
   * // Stow intake when button pressed
   * driver.b().onTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.STOW));
   *
   * // Sequence: deploy, run rollers, then stow
   * Commands.sequence(
   *   IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.PICKUP),
   *   Commands.waitSeconds(1.0),
   *   IntakeCommands.runIntakeRollers(intake).withTimeout(2.0),
   *   IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.STOW)
   * );
   * </pre>
   *
   * @param intake The intake subsystem
   * @param position The target position (IntakePosition.STOW or IntakePosition.PICKUP)
   * @return A command that sets the intake target position and finishes immediately
   */
  public static Command setIntakeGoalPosition(Intake intake, IntakePosition position) {
    return Commands.runOnce(
        () -> {
          intake.setIntakePosition(position);
        },
        intake);
  }

  /**
   * Creates a command to stop the intake roller motors.
   *
   * <p>This command continuously ensures the rollers stay stopped. It's useful as a default command
   * or to explicitly stop the rollers.
   *
   * <p><b>Usage examples:</b>
   *
   * <pre>
   * // Stop rollers when button released
   * driver.leftBumper()
   *   .whileTrue(IntakeCommands.runIntakeRollers(intake))
   *   .onFalse(IntakeCommands.stopIntakeRollers(intake));
   *
   * // As a default command (rollers off when no other command running)
   * intake.setDefaultCommand(IntakeCommands.stopIntakeRollers(intake));
   * </pre>
   *
   * @param intake The intake subsystem
   * @return A command that stops the intake rollers
   */
  public static Command stopIntakeRollers(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopRollers();
        },
        intake);
  }

  /**
   * Creates a command for manual control of the intake pivot using duty cycle output.
   *
   * <p><b>Warning:</b> This bypasses the PID controller! Use this only for:
   *
   * <ul>
   *   <li>Initial testing to verify motor directions
   *   <li>Manual override during debugging
   *   <li>Emergency control if PID fails
   * </ul>
   *
   * <p>For normal operation, use setIntakeGoalPosition() instead, which provides smooth, controlled
   * movement with PID.
   *
   * <p><b>Typical usage during testing:</b>
   *
   * <pre>
   * // Manual control with joystick
   * driver.y().whileTrue(IntakeCommands.setIntakPivotDutyCycle(intake, 0.3));  // Move up slowly
   * driver.x().whileTrue(IntakeCommands.setIntakPivotDutyCycle(intake, -0.3)); // Move down slowly
   * </pre>
   *
   * @param intake The intake subsystem
   * @param output The duty cycle output (-1.0 to +1.0)
   * @return A command that manually controls the intake pivot
   */
  public static Command setIntakPivotDutyCycle(Intake intake, double output) {
    return Commands.run(
        () -> {
          intake.setIntakePivotDutyCucleOutput(output);
        },
        intake);
  }

  /**
   * Creates a command to run the intake roller motors to collect game pieces.
   *
   * <p>This command continuously runs the rollers at the speed defined in
   * IntakeConstants.INTAKE_ACQUIRE_SPEED. Use this when the intake is deployed and you want to
   * collect game pieces from the ground.
   *
   * <p><b>Typical usage:</b>
   *
   * <pre>
   * // Run rollers while button held
   * driver.leftBumper().whileTrue(IntakeCommands.runIntakeRollers(intake));
   * </pre>
   *
   * <p><b>Sequence for collecting game pieces:</b>
   *
   * <ol>
   *   <li>Deploy intake to PICKUP position
   *   <li>Start rollers with this command
   *   <li>Drive toward game piece
   *   <li>Game piece is pulled into hopper
   *   <li>Stop rollers
   * </ol>
   *
   * <p>You might combine this with the position command:
   *
   * <pre>
   * // Deploy and run rollers with one button
   * driver.a().whileTrue(
   *   Commands.parallel(
   *     IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.PICKUP),
   *     IntakeCommands.runIntakeRollers(intake)
   *   )
   * );
   * </pre>
   *
   * @param intake The intake subsystem
   * @return A command that runs the intake rollers
   */
  public static Command runIntakeRollers(Intake intake) {
    return Commands.run(
        () -> {
          intake.turnOnIntakeRollers();
        },
        intake);
  }
}
