package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShooterCharacterizationSnapshot(
        double timestampSec,
        int runId,
        int modeCode,
        int segmentIndex,
        double commandedRpm,
        double commandedVolts,
        double measuredRpm,
        double batteryVolts,
        double flywheelAvgMotorVolts,
        double flywheelTotalSupplyCurrentAmps,
        double flywheelAvgStatorCurrentAmps,
        double electricalPowerWatts,
        double velocityErrorRpm,
        boolean atSetpoint,
        boolean settled,
        boolean aborted) implements StructSerializable {
    public static final Struct<ShooterCharacterizationSnapshot> struct =
            StructGenerator.genRecord(ShooterCharacterizationSnapshot.class);
}
