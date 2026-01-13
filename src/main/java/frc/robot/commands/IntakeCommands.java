package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class IntakeCommands {

  public IntakeCommands() {}

  public static Command setIntakeGoalPosition(Intake intake, IntakePosition position) {
    return Commands.run(
        () -> {
          intake.setIntakePosition(position);
        },
        intake);
  }

  public static Command stopIntakeRollers(Intake intake) {
    return Commands.run(
        () -> {
          intake.stopRollers();
        },
        intake);
  }

  public static Command runIntakeRollers(Intake intake) {
    return Commands.run(
        () -> {
          intake.turnOnIntakeRollers();
        },
        intake);
  }
}
