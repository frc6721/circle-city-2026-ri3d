# Fuel Simulation Integration - Implementation Plan

## Overview
This plan outlines the integration of the FuelSim library into the robot code for visualization of fuel (game piece) intake and shooting. The implementation will support both simulation and real robot operation, with trajectory visualization and virtual hopper management.

---

## Phase 1: Create VirtualHopper Singleton Class

### Step 1.1: Create VirtualHopper.java
**Location:** `src/main/java/frc/lib/VirtualHopper.java`

**Actions:**
- Create a singleton class to manage virtual fuel storage
- Add private constructor to enforce singleton pattern
- Add static instance variable and `getInstance()` method
- Add private integer field `fuelCount` to track current fuel in hopper
- Add getter method `getFuelCount()` that returns current fuel count
- Add setter method `setFuelCount(int count)` to directly set fuel count
- Add method `addFuel()` to increment fuel count by 1
- Add method `removeFuel()` to decrement fuel count by 1 (with check for count >= 1)
- Add method `hasFuel()` that returns boolean (fuelCount > 0)
- Add method `reset()` to clear all fuel
- Add logging of fuel count changes using `Logger.recordOutput("VirtualHopper/FuelCount", fuelCount)`

**Expected Output:** A complete VirtualHopper class ready for integration

---

## Phase 2: Add Fuel Sim Constants to IntakeConstants

### Step 2.1: Add Intake FuelSim Constants
**Location:** `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`

**Actions:**
- Add section comment: `// ==================== FUEL SIM CONSTANTS ====================`
- Add `INTAKE_BOUNDING_BOX_MIN_X` constant using `LoggedNetworkNumber`
  - Intake is 5" deep, sits just outside bumpers (4" thick)
  - Robot center to bumper edge = 26/2 = 13"
  - Intake starts at bumper edge: 13" / 0.0254 m/in = 0.330 m
  - Set value: `0.330` (meters)
- Add `INTAKE_BOUNDING_BOX_MAX_X` constant
  - Extends 5" forward from bumper edge: (13 + 5)" = 18" = 0.457 m
  - Set value: `0.457` (meters)
- Add `INTAKE_BOUNDING_BOX_MIN_Y` constant
  - Intake is 20" wide, centered on robot
  - Left edge: -10" from center = -0.254 m
  - Set value: `-0.254` (meters)
- Add `INTAKE_BOUNDING_BOX_MAX_Y` constant
  - Right edge: +10" from center = +0.254 m
  - Set value: `0.254` (meters)
- Add javadoc comments explaining these are robot-relative coordinates for fuel intake zone
- Add note: "X is forward (positive ahead of robot), Y is left/right (positive to left)"
- Add logging of these constants in the static block

**Notes:**
- Intake height matches bumper height (6" off ground), no Z-axis bounding needed
- FuelSim checks fuel height automatically against bumper height
- Robot-relative coordinates: origin at robot center, X forward, Y left

**Expected Output:** Constants added with correct dimensional values and logged properly

---

## Phase 3: Add Fuel Sim Constants to ShooterConstants

### Step 3.1: Add Shooter FuelSim Constants
**Location:** `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`

**Actions:**
- Add section comment: `// ==================== FUEL SIM CONSTANTS ====================`
- Add `SHOOTER_HEIGHT_FROM_GROUND` constant using `LoggedNetworkNumber`
  - Shooter is 18" off ground = 0.457 m
  - Set value: `0.457` (meters from ground level)
- Add `SHOOTER_FORWARD_OFFSET` constant using `LoggedNetworkNumber`
  - 8" forward from robot center = 0.203 m
  - Set value: `0.203` (meters, positive is forward)
- Add `SHOOTER_SIDE_OFFSET` constant using `LoggedNetworkNumber`
  - Centered left/right
  - Set value: `0.0` (meters, no offset)
- Add `SHOOTER_HOOD_ANGLE` constant using `LoggedNetworkNumber`
  - Fixed hood angle
  - Set value: `30.0` (degrees from horizontal)
- Add `SHOOTER_RPM_THRESHOLD_FOR_LAUNCH` constant using `LoggedNetworkNumber`
  - Minimum speed to trigger visualization
  - Set value: `1000.0` (RPM)
- Add `SHOOTER_LAUNCH_RATE` constant using `LoggedNetworkNumber`
  - Time between virtual launches
  - Set value: `0.5` (seconds, one ball every 0.5 seconds)
- Add `SHOOTER_WHEEL_DIAMETER` constant
  - From existing FLYWHEEL_DIAMETER constant (4" ThriftyBot wheel)
  - Use existing constant or add if needed for velocity conversion
- Add `MAX_HOPPER_CAPACITY` constant (static final int)
  - Set value: `8` (maximum fuel robot can hold)
- Add `STARTING_FUEL_COUNT` constant (static final int)
  - Set value: `8` (fuel when robot is enabled)
- Add `TRAJECTORY_VISUALIZATION_POINTS` constant (static final int)
  - Set value: `25` (number of trajectory points)
- Add javadoc comments explaining each constant and units
- Add logging of these constants in the static block

**Expected Output:** All shooter fuel sim constants defined with correct values and logged

---

## Phase 4: Create ShooterVisualizer Class

### Step 4.1: Create ShooterVisualizer.java
**Location:** `src/main/java/frc/robot/subsystems/shooter/ShooterVisualizer.java`

**Actions:**
- Create class with private fields:
  - `Supplier<Pose2d> poseSupplier` - provides robot pose
  - `Supplier<ChassisSpeeds> fieldSpeedsSupplier` - provides field-relative speeds
  - `Translation3d[] trajectory` - array sized by `TRAJECTORY_VISUALIZATION_POINTS`
  - `double lastLaunchTime` - timestamp of last fuel launch (use `Logger.getRealTimestamp()`)
- Add constructor that takes pose and field speeds suppliers as parameters
  - Initialize trajectory array with size from ShooterConstants
- Add method `launchVel(LinearVelocity linearVel, Angle angle)` that:
  - Gets current robot pose (for rotation)
  - Gets field-relative chassis speeds
  - Calculates horizontal velocity: `horizontalVel = linearVel * cos(angle)`
  - Calculates vertical velocity: `verticalVel = linearVel * sin(angle)`
  - Rotates horizontal velocity by robot heading (converts robot-relative to field-relative)
  - Adds robot's field velocity to projectile velocity
  - Returns `Translation3d` representing initial fuel velocity (field-relative)
  - Note: Use `Math.cos(angle.in(Radians))` and `Math.sin(angle.in(Radians))`
- Add method `launchFuel(LinearVelocity linearVel, Angle angle)` that:
  - Checks if VirtualHopper has fuel (return early if not)
  - Removes one fuel from VirtualHopper
  - Gets robot pose from supplier (as Pose2d, convert to Pose3d)
  - Calculates shooter 3D position using:
    - Offset from robot center: `(SHOOTER_FORWARD_OFFSET, SHOOTER_SIDE_OFFSET)`
    - Rotate offset by robot heading
    - Add height: `SHOOTER_HEIGHT_FROM_GROUND`
    - Add to robot position
  - Calculates launch velocity using `launchVel()`
  - Spawns fuel in FuelSim: `FuelSim.getInstance().spawnFuel(position, velocity)`
  - Updates last launch time: `lastLaunchTime = Logger.getRealTimestamp() / 1_000_000.0` (microseconds to seconds)
- Add method `updateTrajectory(LinearVelocity linearVel, Angle angle)` that:
  - Gets current shooter position (same calculation as in launchFuel)
  - Calculates trajectory velocity using `launchVel()`
  - Loops through trajectory array (i = 0 to TRAJECTORY_VISUALIZATION_POINTS):
    - Time for point: `t = i * 0.04` (0.04s between points for ~1 second total)
    - X position: `x = initialX + velocityX * t`
    - Y position: `y = initialY + velocityY * t`
    - Z position: `z = initialZ + velocityZ * t - 0.5 * 9.81 * t²` (gravity)
    - Store as `trajectory[i] = new Translation3d(x, y, z)`
  - Logs trajectory: `Logger.recordOutput("Shooter/Trajectory", trajectory)`
- Add method `canLaunch()` that:
  - Gets current time: `Logger.getRealTimestamp() / 1_000_000.0`
  - Returns true if `(currentTime - lastLaunchTime) >= SHOOTER_LAUNCH_RATE`
- Add helper method `convertRPMToLinearVelocity(AngularVelocity rpm)` that:
  - Converts RPM to linear velocity at wheel edge
  - Formula: `linearVel = rpm * PI * diameter`
  - Use SHOOTER_WHEEL_DIAMETER constant
  - Return as `LinearVelocity`

**Expected Output:** Complete ShooterVisualizer class with trajectory calculation and proper velocity conversion

---

## Phase 5: Modify Intake Subsystem

### Step 5.1: Add FuelSim Integration to Intake
**Location:** `src/main/java/frc/robot/subsystems/intake/Intake.java`

**Actions:**
- Add method `simIntakeFuel()` that:
  - Checks if VirtualHopper has capacity (using MAX_HOPPER_CAPACITY constant)
  - If has capacity, calls `VirtualHopper.getInstance().addFuel()`
  - Logs intake event
- Add method `isDeployed()` that:
  - Returns true if intake position is PICKUP
  - Returns false if intake position is STOW
- Add method `isAtTarget()` that:
  - Calculates absolute difference between current angle and target angle
  - Returns true if difference is less than `INTAKE_PIVOT_DEADBAND`
  - Returns false otherwise
- Add method `canIntakeFuel()` that:
  - Returns true if: `isDeployed() && isAtTarget() && VirtualHopper.getInstance().getFuelCount() < MAX_HOPPER_CAPACITY`

**Expected Output:** Intake subsystem ready to interface with FuelSim

---

## Phase 6: Modify Shooter Subsystem - Add Dependencies

### Step 6.1: Add Pose and Speed Suppliers to Shooter
**Location:** `src/main/java/frc/robot/subsystems/shooter/Shooter.java`

**Actions:**
- Add private fields:
  - `Supplier<Pose2d> poseSupplier`
  - `Supplier<ChassisSpeeds> fieldSpeedsSupplier`
  - `ShooterVisualizer visualizer`
  - `double timeAtTargetSpeed` - tracks when flywheel reached target
- Modify constructor signature to accept:
  - `ShooterIO shooterIO` (existing)
  - `Supplier<Pose2d> poseSupplier` (new)
  - `Supplier<ChassisSpeeds> fieldSpeedsSupplier` (new)
- In constructor:
  - Store pose and speed suppliers
  - Create ShooterVisualizer: `visualizer = new ShooterVisualizer(poseSupplier, fieldSpeedsSupplier)`
  - Initialize timeAtTargetSpeed to 0

**Expected Output:** Shooter class prepared for visualization

---

## Phase 7: Modify Shooter Subsystem - Add Fuel Launching Logic

### Step 7.1: Add Fuel Launch Methods to Shooter
**Location:** `src/main/java/frc/robot/subsystems/shooter/Shooter.java`

**Actions:**
- Add method `shouldVisualizeLaunch()` that returns boolean:
  - Check if flywheels are at target speed using existing `areFlywheelsAtTargetSpeed()`
  - Check if target flywheel speed (in RPM) is above `SHOOTER_RPM_THRESHOLD_FOR_LAUNCH`
    - Convert `_targetFlywheelSpeed` to RPM for comparison
  - Check if VirtualHopper has fuel
  - Check if enough time has passed since last launch using `visualizer.canLaunch()`
  - Return true only if all conditions met
  - Note: This is for visualization only, does NOT affect actual robot shooting
- Add method `visualizeFuelLaunch()` that:
  - Checks if `shouldVisualizeLaunch()` is true (return early if not)
  - Gets current measured flywheel speed from `_shooterInputs._flywheelMotorVelocity`
  - Converts measured speed to linear velocity (uses ShooterVisualizer helper)
  - Gets hood angle from `ShooterConstants.SHOOTER_HOOD_ANGLE`
  - Calls `visualizer.launchFuel(linearSpeed, angle)`
  - Note: Uses MEASURED speed, not target speed
- Modify `periodic()` method:
  - After existing logging code, add:
  - Convert current measured speed to linear velocity
  - Get hood angle from constants
  - Update trajectory visualization: `visualizer.updateTrajectory(linearSpeed, hoodAngle)`
    - This happens every loop for real-time updates
  - Call `visualizeFuelLaunch()` to spawn fuel if conditions met
  - Log `shouldVisualizeLaunch()` result for debugging
- Note: Shooter visualization is independent of feeder operation
  - Driver still controls shooting with existing buttons
  - Visualization just shows what's happening based on flywheel state

**Expected Output:** Shooter visualizes fuel launching automatically when at speed, without changing driver controls

---

## Phase 8: Configure FuelSim in RobotContainer

### Step 8.1: Add Robot Dimensions Constants
**Location:** Create new file `src/main/java/frc/robot/Dimensions.java`

**Actions:**
- Create public class with public static final constants:
  - `ROBOT_WIDTH` - full width with bumpers
    - Robot frame: 26"
    - Bumpers on each side: 4" × 2 = 8"
    - Total: 26" + 8" = 34" = 0.8636 m
    - `Distance ROBOT_WIDTH = Inches.of(34.0)`
  - `ROBOT_LENGTH` - full length with bumpers
    - Same calculation as width
    - Total: 34" = 0.8636 m
    - `Distance ROBOT_LENGTH = Inches.of(34.0)`
  - `BUMPER_HEIGHT` - height to top of bumpers
    - Bumpers are 6" off ground at top
    - `Distance BUMPER_HEIGHT = Inches.of(6.0)`
- Add javadoc comments explaining each dimension
- Add note about robot being square (26×26 frame)
- Log constants in static block using `Logger.recordOutput()`

**Expected Output:** Robot dimensions defined accurately in one place

---

### Step 8.2: Add FuelSim Configuration Method
**Location:** `src/main/java/frc/robot/RobotContainer.java`

**Actions:**
- Add import statements:
  - `import frc.lib.feulSim.FuelSim;`
  - `import frc.lib.VirtualHopper;`
  - `import frc.robot.Dimensions;`
  - `import static edu.wpi.first.units.Units.Meters;`
- Add private method `configureFuelSim()` that:
  - Gets FuelSim instance: `FuelSim instance = FuelSim.getInstance()`
  - Spawns starting fuel: `instance.spawnStartingFuel()`
  - Registers robot with FuelSim:
    ```java
    instance.registerRobot(
        Dimensions.ROBOT_WIDTH.in(Meters),
        Dimensions.ROBOT_LENGTH.in(Meters),
        Dimensions.BUMPER_HEIGHT.in(Meters),
        drive::getPose,
        drive::getFieldRelativeSpeeds  // New method to be added to Drive
    )
    ```
  - Registers intake with FuelSim:
    ```java
    instance.registerIntake(
        IntakeConstants.INTAKE_BOUNDING_BOX_MIN_X.get(),
        IntakeConstants.INTAKE_BOUNDING_BOX_MAX_X.get(),
        IntakeConstants.INTAKE_BOUNDING_BOX_MIN_Y.get(),
        IntakeConstants.INTAKE_BOUNDING_BOX_MAX_Y.get(),
        intake::canIntakeFuel,  // BooleanSupplier - checks if intake can collect
        intake::simIntakeFuel   // Runnable - called when fuel collected
    )
    ```
  - Starts FuelSim: `instance.start()`
  - Adds SmartDashboard button to reset fuel and hopper:
    ```java
    SmartDashboard.putData(
        Commands.runOnce(() -> {
            FuelSim.getInstance().clearFuel();
            FuelSim.getInstance().spawnStartingFuel();
            VirtualHopper.getInstance().reset();
        })
        .withName("Reset Fuel")
        .ignoringDisable(true)
    );
    ```
- Call `configureFuelSim()` at end of RobotContainer constructor (after configureButtonBindings)
- Wrap in try-catch to handle any FuelSim initialization errors gracefully

**Expected Output:** FuelSim fully configured and integrated with proper error handling

---

## Phase 9: Add getFieldRelativeSpeeds() Method to Drive

### Step 9.1: Create Field-Relative Speeds Method
**Location:** `src/main/java/frc/robot/subsystems/drive/Drive.java`

**Actions:**
- Check existing `getChassisSpeeds()` method (currently private, marked `@AutoLogOutput`)
  - This method returns measured chassis speeds from module states
  - Currently logs as "SwerveChassisSpeeds/Measured"
  - Returns robot-relative speeds (not field-relative)
- Add new public method `getFieldRelativeSpeeds()` that:
  - Gets current robot pose: `Pose2d pose = getPose()`
  - Calls existing `getChassisSpeeds()` to get robot-relative speeds
  - Transforms to field-relative frame:
    ```java
    ChassisSpeeds robotRelative = getChassisSpeeds();
    ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelative,
        pose.getRotation()
    );
    return fieldRelative;
    ```
  - Add javadoc explaining this returns field-relative speeds for FuelSim
  - Note: Field-relative means vx/vy are in field coordinates, not robot coordinates
- Do NOT modify the existing private `getChassisSpeeds()` method
- Do NOT change the `@AutoLogOutput` annotation or logging

**Expected Output:** Drive subsystem provides field-relative speeds via new public method

---

## Phase 10: Update Shooter Instantiation in RobotContainer

### Step 10.1: Pass Dependencies to Shooter Constructor
**Location:** `src/main/java/frc/robot/RobotContainer.java`

**Actions:**
- Find all three locations where Shooter is instantiated (REAL, SIM, REPLAY modes)
- For REAL mode, change:
  ```java
  shooter = new Shooter(new RealShooterIO());
  ```
  to:
  ```java
  shooter = new Shooter(new RealShooterIO(), drive::getPose, drive::getFieldSpeeds);
  ```
- For SIM mode, change similarly
- For REPLAY mode, provide no-op suppliers:
  ```java
  shooter = new Shooter(
      new ShooterIO() {},
      () -> new Pose2d(),
      () -> new ChassisSpeeds()
  );
  ```

**Expected Output:** Shooter receives required dependencies in all modes

---

## Phase 11: Initialize Virtual Hopper on Enable

### Step 11.1: Reset Fuel on Robot Enable
**Location:** `src/main/java/frc/robot/Robot.java`

**Actions:**
- Find the `teleopInit()` method
- Add at the beginning:
  ```java
  VirtualHopper.getInstance().setFuelCount(ShooterConstants.STARTING_FUEL_COUNT);
  ```
- Find the `autonomousInit()` method
- Add at the beginning:
  ```java
  VirtualHopper.getInstance().setFuelCount(ShooterConstants.STARTING_FUEL_COUNT);
  ```

**Expected Output:** Robot starts each match with 8 fuel

---

## Phase 12: Update Robot.java for Simulation Periodic

### Step 12.1: Add FuelSim Update to Simulation
**Location:** `src/main/java/frc/robot/Robot.java`

**Actions:**
- Find the `simulationPeriodic()` method
- Add call to update FuelSim:
  ```java
  FuelSim.getInstance().updateSim();
  ```

**Expected Output:** FuelSim updates during simulation

---

## Phase 13: Testing and Tuning Constants

### Step 13.1: Test Intake Bounding Box
**Actions:**
- Deploy code to sim
- Enable robot in simulation
- Drive near fuel on ground
- Deploy intake to PICKUP position
- Verify fuel is collected when intake is in correct position
- Tune bounding box constants if needed
- Verify fuel count increases in logs

### Step 13.2: Test Shooter Visualization
**Actions:**
- With fuel in hopper, spin up shooter
- Verify trajectory appears in AdvantageScope
- Tune hood angle and shooter height/offset if trajectory looks wrong
- Verify trajectory matches where fuel actually goes when launched

### Step 13.3: Test Automatic Launching
**Actions:**
- Spin up shooter to target speed
- Verify fuel launches automatically after delay
- Tune launch delay constant if needed
- Verify fuel velocity and trajectory are realistic
- Check that hopper count decreases correctly

**Expected Output:** Fully functional fuel simulation with proper visualization

---

## Phase 14: Documentation and Cleanup

### Step 14.1: Add Javadoc Comments
**Actions:**
- Review all new methods and classes
- Ensure all public methods have comprehensive javadoc
- Add code examples where helpful
- Document units for all physical constants

### Step 14.2: Add Educational Comments
**Actions:**
- Add comments explaining physics calculations in trajectory code
- Explain coordinate system transformations
- Document the relationship between shooter speed, angle, and trajectory

**Expected Output:** Well-documented, student-friendly code

---

## Testing Checklist

- [ ] VirtualHopper correctly tracks fuel count
- [ ] Intake bounding box positioned correctly relative to robot
- [ ] Fuel is added to hopper when intake runs over fuel
- [ ] Fuel does not exceed max capacity
- [ ] Shooter visualizer creates correct trajectory
- [ ] Trajectory visualization appears in AdvantageScope
- [ ] Fuel launches when shooter at speed
- [ ] Launch respects delay after reaching target speed
- [ ] Fuel velocity accounts for robot motion
- [ ] Robot starts with 8 fuel when enabled
- [ ] Reset button clears and respawns fuel
- [ ] FuelSim works in both sim and real modes
- [ ] All constants logged to AdvantageKit
- [ ] Code compiles without errors
- [ ] No runtime exceptions during operation

---

## Key Design Decisions

1. **VirtualHopper as Singleton:** Ensures single source of truth for fuel count across entire robot code

2. **Dependency Injection for Pose/Speeds:** Keeps shooter decoupled from drive subsystem, enables testing and simulation

3. **Automatic Launching:** Shooter launches when conditions met, simplifying driver controls

4. **Tunable Constants:** All physical parameters as LoggedNetworkNumbers for easy tuning without redeployment

5. **Mode-Independent Visualization:** Trajectory visualization works in both sim and real, helping drivers understand shooter behavior

6. **Educational Comments:** Extensive documentation helps students understand physics and coordinate transformations

---

---

## Summary of Specifications

### Robot Physical Specifications
- **Frame Dimensions:** 26" × 26" (square frame)
- **Bumpers:** 4" thick on all sides, 6" tall (top of bumper is 6" off ground)
- **Total Robot Dimensions:** 34" × 34" with bumpers (0.8636 m)

### Shooter Configuration
- **Height:** 18" off ground (0.457 m)
- **Forward Offset:** 8" ahead of robot center (0.203 m)
- **Side Offset:** 0" (centered left/right)
- **Hood Angle:** 30° (fixed, not adjustable)
- **Launch Threshold:** Target speed > 1000 RPM
- **Launch Rate:** One fuel every 0.5 seconds
- **Velocity Source:** Use ACTUAL MEASURED flywheel speed (not target)
- **Velocity Conversion:** Convert RPM to linear velocity using 4" wheel diameter

### Intake Configuration
- **Width:** 20" (10" on each side of center)
- **Depth:** 5" forward
- **Height:** Matches bumper height (6" off ground)
- **Position:** Just outside bumpers (starts at bumper edge, extends forward)
- **Bounding Box (robot-relative):**
  - X: 0.330 m to 0.457 m (bumper edge to 5" forward)
  - Y: -0.254 m to 0.254 m (±10" from center)

### Visualization Behavior
- **Launch Trigger:** Automatic visualization when:
  1. Flywheels at target speed
  2. Target speed > 1000 RPM
  3. Virtual hopper has fuel
  4. Enough time passed since last launch (0.5s)
- **Driver Control:** Driver shooting logic UNCHANGED
  - Driver still controls with existing buttons
  - Visualization shows what's happening based on flywheel state
- **Trajectory Display:**
  - 25 points
  - Updates in REAL-TIME (every periodic loop)
  - Always visible when shooter is spinning

### Technical Decisions
- **Field-Relative Speeds:** New `getFieldRelativeSpeeds()` method in Drive subsystem
- **Speed Source:** Measured speed from sensors, not commanded speed
- **Independent Operation:** Shooter visualization independent of feeder operation

---

## Common Pitfalls to Avoid

- Forgetting to call `FuelSim.getInstance().start()` after configuration
- Not transforming velocities to field-relative frame
- Using wrong coordinate system for bounding box (robot-relative vs field-relative)
- Not checking hopper capacity before adding fuel
- Launching fuel without checking if hopper has fuel
- Not resetting timeAtTargetSpeed after launch
- Forgetting to update FuelSim in `simulationPeriodic()`
- Providing robot-relative speeds instead of field-relative to FuelSim
