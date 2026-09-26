package com.team9470.commands;

import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Wheel Radius Characterization (based on FRC 6328 Mechanical Advantage).
 *
 * <p>
 * Spins the robot slowly in place and uses the gyro yaw accumulation
 * compared against the drive encoder deltas to back-compute the effective
 * wheel radius. This accounts for wheel wear and carpet compression.
 *
 * <p>
 * <b>Usage:</b> Bind to a held button in teleop and let the robot complete at
 * least one full rotation. Release the button to stop. The measured wheel
 * radius is printed to the console.
 */
public class WheelRadiusCharacterization extends Command {

    /** Distance from robot center to each swerve module (modules at ±11 in). */
    private static final double DRIVE_BASE_RADIUS = Math.hypot(Units.inchesToMeters(10.875), Units.inchesToMeters(10.875));

    /** Slow rotation speed (rad/s). Keep low to avoid wheel slip. */
    private static final double CHARACTERIZATION_SPEED = 1.0;

    /**
     * Configured wheel radius used by the drive encoders (must match
     * TunerConstants).
     */
    private static final double CONFIGURED_WHEEL_RADIUS = Units.inchesToMeters(1.95);

    private final Swerve swerve;

    private double[] startWheelPositions;
    private double lastGyroYawRad;
    private double gyroYawAccumRad;
    private double currentEffectiveRadius;
    private double effectiveRadiusSum;
    private int effectiveRadiusSamples;

    public WheelRadiusCharacterization(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startWheelPositions = swerve.getWheelRadiusCharacterizationPosition();
        lastGyroYawRad = swerve.getGyroHeading().getRadians();
        gyroYawAccumRad = 0.0;
        currentEffectiveRadius = 0.0;
        effectiveRadiusSum = 0.0;
        effectiveRadiusSamples = 0;
    }

    @Override
    public void execute() {
        // Command a pure rotation
        swerve.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, CHARACTERIZATION_SPEED));

        // Accumulate gyro yaw (handle wraparound)
        double currentGyroYawRad = swerve.getGyroHeading().getRadians();
        gyroYawAccumRad += MathUtil.angleModulus(currentGyroYawRad - lastGyroYawRad);
        lastGyroYawRad = currentGyroYawRad;

        // Average wheel position delta across all modules
        double[] currentPositions = swerve.getWheelRadiusCharacterizationPosition();
        double totalDelta = 0.0;
        for (int i = 0; i < currentPositions.length; i++) {
            totalDelta += Math.abs(currentPositions[i] - startWheelPositions[i]);
        }
        double avgWheelDelta = totalDelta / currentPositions.length;

        // Back-compute effective wheel radius.
        // Encoder reports meters using configured radius. Convert to wheel radians,
        // then: trueRadius = |gyroAccum| * driveBaseRadius / avgWheelRadians
        double absGyroAccum = Math.abs(gyroYawAccumRad);
        if (absGyroAccum > 0.1 && avgWheelDelta > 0.01) {
            double avgWheelRadians = avgWheelDelta / CONFIGURED_WHEEL_RADIUS;
            double instantEffectiveRadius = (absGyroAccum * DRIVE_BASE_RADIUS) / avgWheelRadians;
            effectiveRadiusSum += instantEffectiveRadius;
            effectiveRadiusSamples++;
            currentEffectiveRadius = effectiveRadiusSum / effectiveRadiusSamples;
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());

        if (currentEffectiveRadius > 0.0) {
            System.out.println("==============================================");
            System.out.println("  WHEEL RADIUS CHARACTERIZATION RESULT");
            System.out.println("----------------------------------------------");
            System.out.printf("  Effective wheel radius: %.6f m%n", currentEffectiveRadius);
            System.out.printf("  Effective wheel radius: %.4f in%n", Units.metersToInches(currentEffectiveRadius));
            System.out.printf("  Radius samples averaged: %d%n", effectiveRadiusSamples);
            System.out.printf("  Gyro accumulation: %.2f deg (%.2f rotations)%n",
                    Math.toDegrees(Math.abs(gyroYawAccumRad)),
                    Math.abs(gyroYawAccumRad) / (2.0 * Math.PI));
            System.out.println("==============================================");
        } else {
            System.out.println("Wheel radius characterization: not enough data. "
                    + "Ensure the robot completed at least one full rotation.");
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Disable the robot to end
    }
}
