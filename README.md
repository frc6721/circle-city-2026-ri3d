# circle-city-2026-ri3d

Robot in 3 Days (RI3D) code for Circle City Robots

## Overview

Our codebase is built upon the **Spark Swerve Template** from team 6328 and makes use of their AdvantageKit logging framework. We intend to release a simplified version of the code without the logging framework for teams who are new to FRC or have less programming resources.

> **Note**: Both versions of the code use the `Spark Swerve Template` from 6328 as it is our recommended starting point for teams using an all-REV swerve drivetrain.

> **Finding What to Change**: To assist with tuning your swerve and following the setup instructions, we have added `TODO` comments throughout the codebase marking locations where you may need to modify code. Search for `TODO` in your VScode to find all these locations quickly.

## About AdvantageKit

AdvantageKit is a comprehensive logging framework for FRC. However, we focus on its **swerve template project** because it is:
- Easy to understand
- Widely used at events
- Well-documented

> **Important for Rookies**: For teams new to FRC or with limited programming resources, we do **NOT** recommend using AdvantageKit for your non-drivetrain subsystems. Just use the normal [Command Based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) structure for other mechanisms.

## Getting Started: Setting Up Your Swerve Drive

Start by reading the [instructions](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template) for the Spark Swerve Template. This will guide you through editing code and tuning your drivetrain.

> **Hardware Compatibility**: If you are using NEO motors for both turn and drive with Spark MAXs, you will not need to make code changes to the IO implementation.

### Setup Steps

Their documentation is comprehensive, so we won't repeat it here, but the high-level overview is:

#### 1. Update Hardware Configuration
- Update CAN IDs for your motor controllers
- Update swerve module constants (free speed, gear reductions, etc.)
- Update physical robot property constants (MOI, track width, wheel diameter, etc.)
- **If using a gyro other than NavX or Pigeon 2**: You will need to write your own GyroIO implementation. See the Advanatge Kit documentation for more on the concepts behind the IO layers. You can also reach out to us on our cheif delphi build thread for assistance.

#### 2. Initial Testing & Calibration
Deploy code and perform initial testing:
1. **Verify motors and encoders are not inverted**
   - Drive forward and ensure all wheels spin forward
   - Turn modules and verify encoder readings increase in the correct direction
2. **Update module rotation offsets**
   - Align all wheels forward
   - Record absolute encoder readings
   - Update zero rotation constants in `DriveConstants.java`

#### 3. Tune PID Controllers
- Tune Drive PID for velocity control
- Tune Turn PID for position control

> **Success!** After completing these steps, you have a working swerve drive that can drive on the field at competition!

#### 4. (Optional) Feedforward Characterization

For better autonomous performance, you should also tune the Feedforward Characterization. This is documented in the [instructions](https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template).

> **Competition Ready**: With steps 1-3 complete, you will be able to drive on the field. Feedforward tuning improves auto performance but is not required for basic operation.

## Hardware Notes

### Using Spark Flex Controllers

If you are using Spark Flex motor controllers instead of Spark Max, you'll need to update the IO implementation:

1. Change imports from `SparkMax` to `SparkFlex`
2. Change `SparkMaxConfig` to `SparkFlexConfig`
3. Update the constructor instantiation

See `ModuleIOSpark.java` for `TODO` comments marking these locations.

### Using a Different Gyro

The template code supports NavX and Pigeon 2 gyros by default. If you are using a different gyro:

1. Create a new IO implementation for your gyro (e.g., `GyroIOPigeon1.java`)
2. Update the `Drive` subsystem constructor to use your new implementation
3. Refer to your gyro's vendor documentation for API details


## Project Structure

This codebase follows the **hardware abstraction pattern** from AdvantageKit:

- **`ModuleIO.java`**: Interface defining all swerve module operations
- **`ModuleIOSpark.java`**: Implementation for REV motor controllers
- **`ModuleIOSim.java`**: Simulation implementation for testing without hardware
- **`Drive.java`**: High-level drivetrain subsystem logic

> **Why This Matters**: This pattern allows you to test code in simulation and easily switch between different hardware configurations without changing your subsystem logic.

## Additional Resources

- [WPILib Documentation](https://docs.wpilib.org/)
- [REVLib Documentation](https://docs.revrobotics.com/revlib)
- [AdvantageKit Documentation](https://docs.advantagekit.org/)
- [6328 GitHub](https://github.com/Mechanical-Advantage)

## Questions?

If you have questions about this code or swerve drive in general, please reach out to Circle City Robots mentors or post in the [Chief Delphi forums](https://www.chiefdelphi.com/t/circle-city-circuits-2026-ri3d-presented-by-denkbots/509780/5).
