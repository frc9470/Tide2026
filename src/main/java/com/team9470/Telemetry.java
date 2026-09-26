package com.team9470;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.telemetry.structs.DriveStatusSnapshot;
import edu.wpi.first.wpilibj.Timer;

/**
 * Publishes drivetrain telemetry using canonical NT4 struct topics.
 */
public class Telemetry {
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    public Telemetry(double maxSpeedMetersPerSecond) {
        // Phoenix SignalLogger is intentionally left off during normal boots.
        // We are not using .hoot logs for matches right now, and enabling it
        // continuously just consumes robot storage.
    }

    public void telemeterize(SwerveDriveState state) {
        double odometryFrequencyHz = state.OdometryPeriod > 0.0 ? 1.0 / state.OdometryPeriod : 0.0;

        telemetry.publishDrivePose(Timer.getTimestamp(), state.Pose);
        telemetry.publishDriveSpeeds(Timer.getTimestamp(), state.Speeds);
        telemetry.publishDriveModuleStates(state.ModuleStates);
        telemetry.publishDriveModuleTargets(state.ModuleTargets);
        telemetry.publishDriveModulePositions(state.ModulePositions);
        telemetry.publishDriveStatus(new DriveStatusSnapshot(state.Timestamp, odometryFrequencyHz, state.OdometryPeriod));
    }
}
