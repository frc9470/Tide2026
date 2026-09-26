# Tide clone 2026 robot code

A 2026 REBUILT robot project seeded from Team 9470's `Alpha2026` `main` at `404fa75`.
It keeps the command-based project layout, CTRE swerve implementation, Choreo support,
PhotonVision pose estimation, field constants, telemetry, and shared utilities. Alpha's
shooter, hopper, intake, superstructure, controller bindings, and autonomous paths were
removed because they describe a different robot.

## Bring-up

1. Check `TunerConstants.java` against the clone's actual CAN IDs, encoder offsets,
   gearing, wheel radius, Pigeon orientation, dimensions, and current limits. The values
   currently in that file are from Alpha2026. Check `Constants.RobotGeometry` too.
2. Set `HardwareConfig.DRIVETRAIN_VERIFIED` to `true` only after that check. Until then,
   the program runs without constructing motor controllers. Once enabled, the driver
   Xbox controller provides field-centric drive; X holds wheel lock. Back in test mode
   runs wheel radius characterization.
3. Set camera names in `Vision.java` and measure camera transforms in
   `VisionConstants.java`. Check the 2026 AprilTag layout. Then set
   `HardwareConfig.VISION_VERIFIED` to `true` to feed vision poses into odometry.
4. Implement clone-specific mechanisms in the existing `subsystems/intake`,
   `subsystems/hopper`, and `subsystems/shooter` packages. Add coordination in
   `Superstructure` and bindings in `RobotContainer`. Put the new CAN/DIO assignments
   in `Ports.java`.
5. Add Choreo trajectories under `src/main/deploy/choreo` and register autonomous
   commands in `RobotContainer`.

`./gradlew compileJava` checks that the project builds. Compilation does not verify
motor direction, sensor offsets, camera placement, or mechanism wiring.
