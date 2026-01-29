// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.VirtualHopper;
import frc.lib.feulSim.FuelSim;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.io.FeederIO;
import frc.robot.subsystems.feeder.io.RealFeederIO;
import frc.robot.subsystems.feeder.io.SimFeederIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.RealIntakeIO;
import frc.robot.subsystems.intake.io.SimIntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.io.RealShooterIO;
import frc.robot.subsystems.shooter.io.ShooterIO;
import frc.robot.subsystems.shooter.io.SimShooterIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Feeder feeder;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                // TODO: Change to GyroIONavX if using a NavX
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        intake = new Intake(new RealIntakeIO());
        // Pass pose and field speed suppliers for FuelSim integration
        shooter = new Shooter(new RealShooterIO(), drive::getPose, drive::getFieldRelativeSpeeds);
        feeder = new Feeder(new RealFeederIO());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        // Use simulation IO implementations with physics simulation
        intake = new Intake(new SimIntakeIO());
        // Pass pose and field speed suppliers for FuelSim integration
        shooter = new Shooter(new SimShooterIO(), drive::getPose, drive::getFieldRelativeSpeeds);
        feeder = new Feeder(new SimFeederIO());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        // For replay, use empty IO implementations
        intake = new Intake(new IntakeIO() {});
        // For replay, provide no-op suppliers (no live robot state)
        shooter = new Shooter(new ShooterIO() {}, () -> new Pose2d(), () -> new ChassisSpeeds());
        feeder = new Feeder(new FeederIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData(
        "Stow Intake", IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.STOW));
    SmartDashboard.putData(
        "Deploy Intake", IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.PICKUP));

    // Configure FuelSim for game piece visualization
    configureFuelSim();
  }

  /**
   * Configures the FuelSim system for game piece simulation and visualization.
   *
   * <p>FuelSim allows us to visualize:
   *
   * <ul>
   *   <li>Game pieces (fuel) on the field
   *   <li>Intake collecting fuel when driven over
   *   <li>Shooter trajectory prediction
   *   <li>Launched fuel following realistic physics
   * </ul>
   *
   * <p>This method:
   *
   * <ol>
   *   <li>Spawns starting fuel on the field
   *   <li>Registers robot dimensions for collision detection
   *   <li>Registers intake bounding box for fuel collection
   *   <li>Adds a dashboard button to reset fuel positions
   *   <li>Starts the simulation
   * </ol>
   */
  private void configureFuelSim() {
    try {
      FuelSim fuelSim = FuelSim.getInstance();

      // Spawn initial fuel on the field (neutral zone and depots)
      fuelSim.spawnStartingFuel();

      // Register robot with FuelSim for collision detection
      // Robot pushes fuel out of the way when driving
      fuelSim.registerRobot(
          RobotDimensions.ROBOT_WIDTH.in(Meters),
          RobotDimensions.ROBOT_LENGTH.in(Meters),
          RobotDimensions.BUMPER_HEIGHT.in(Meters),
          drive::getPose,
          drive::getFieldRelativeSpeeds);

      // Register intake with FuelSim
      // Fuel inside the bounding box will be "collected" when canIntakeFuel() returns true
      fuelSim.registerIntake(
          RobotDimensions.ROBOT_WIDTH.div(2).unaryMinus().minus(IntakeConstants.INTAKE_WIDTH.div(2)).in(Meters),
        -RobotDimensions.ROBOT_WIDTH.div(2).in(Meters), // Offset intake box to back of robot 
          -IntakeConstants.INTAKE_LENGTH.div(2).in(Meters),
          IntakeConstants.INTAKE_LENGTH.div(2).in(Meters),
          intake::canIntakeFuel, // BooleanSupplier - checks if intake can collect
          intake::simIntakeFuel); // Runnable - called when fuel is collected

      // Start the simulation (updateSim must still be called each loop)
      fuelSim.start();

      // Add dashboard button to reset fuel and hopper
      SmartDashboard.putData(
          "Reset Fuel",
          Commands.runOnce(
                  () -> {
                    // Clear all fuel from the field
                    FuelSim.getInstance().clearFuel();
                    // Respawn fuel in starting positions
                    FuelSim.getInstance().spawnStartingFuel();
                    // Reset virtual hopper to empty
                    VirtualHopper.getInstance().reset();
                  })
              .withName("Reset Fuel")
              .ignoringDisable(true));

    } catch (Exception e) {
      // Log error but don't crash - FuelSim is for visualization only
      System.err.println("FuelSim initialization failed: " + e.getMessage());
      e.printStackTrace();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // real controller
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX()));

    // sim controller in MAC os
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -(controller.getRightTriggerAxis())));

    // Always run the flywheels a little bit during the match so they can spin up quicker when we
    // need them
    // shooter.setDefaultCommand(ShooterCommands.runFlywheelsAtIdle(shooter));

    controller
        .leftBumper()
        .whileTrue(IntakeCommands.runIntakeRollers(intake))
        .onFalse(IntakeCommands.stopIntakeRollers(intake));

    // A button: Move intake to PICKUP position (down)
    controller.a().whileTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.PICKUP));

    // B button: Move intake to STOW position (up)
    controller.b().whileTrue(IntakeCommands.setIntakeGoalPosition(intake, IntakePosition.STOW));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller
    //     .x()
    //     .onTrue(FeederCommands.runFeederAtPercentOutput(feeder, .5))
    //     .onFalse(FeederCommands.stopFeeder(feeder));

    // Right bumper is for the real controller
    // y() button on xbox on mac os
    controller
        .y()
        .onTrue(
            ShooterCommands.setFlywheelTargetSpeed(
                    shooter, RevolutionsPerSecond.of(4000 / 60.0)) // 4000 RPM
                .andThen(ShooterCommands.waitForFlywheelsToReachSpeed(shooter).withTimeout(2))
                .andThen(FeederCommands.runFeederAtPercentOutput(feeder, 0.75).repeatedly()))
        .onFalse(
            FeederCommands.stopFeeder(feeder).andThen(ShooterCommands.runFlywheelsAtIdle(shooter)));

    // Reset gyro to 0° when left dpad is pressed
    controller
        .pov(270)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // controller
    //     .x()
    //     .onTrue(
    //         ShooterCommands.setFlywheelTargetSpeed(shooter, RevolutionsPerSecond.of(3500 /
    // 60.0)))
    //     .onFalse(
    //         ShooterCommands.setFlywheelTargetSpeed(shooter, RevolutionsPerSecond.of(0 / 60.0)));

    // Used for shooter characterization routines. Not for normal use.
    // Original X binding replaced with shooter characterization bindings
    // controller
    //     .x()
    //     .onTrue(Commands.run(() -> shooter.setFlyWheelDutyCycle(0.015), shooter))
    //     .onFalse(Commands.run(() -> shooter.setFlyWheelDutyCycle(0.0), shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
