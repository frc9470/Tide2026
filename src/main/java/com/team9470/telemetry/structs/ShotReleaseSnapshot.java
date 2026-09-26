package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record ShotReleaseSnapshot(
        boolean canFire,
        boolean releaseAllowed,
        boolean releaseStartedThisHold,
        boolean shotValid,
        boolean hoodHomed,
        boolean hoodAtSetpoint,
        double hoodErrorDeg,
        double hoodToleranceDeg,
        boolean flywheelAtSetpoint,
        boolean flywheelReadyForRelease,
        double flywheelErrorRps,
        double flywheelToleranceRps,
        boolean headingAligned,
        double headingErrorDeg,
        double headingToleranceDeg,
        boolean sotmFireSafe,
        boolean sotmSafetyActive,
        double shooterFieldSpeedMps,
        double shooterFieldAccelMps2,
        double sotmAccelLimitMps2,
        double robotOmegaDegPerSec,
        double sotmOmegaLimitDegPerSec,
        boolean timedAutoArmCandidate,
        boolean armedDuringInactiveThisHold,
        boolean timedReleaseStarted,
        boolean timingKnown,
        boolean zoneActive,
        boolean topSensorBlocked,
        boolean releaseWindowOpen,
        double zoneRemainingSec,
        double launchLeadSec,
        boolean timedReleaseAllowed,
        boolean shotIsFeedMode) implements StructSerializable {
    public static final Struct<ShotReleaseSnapshot> struct = StructGenerator.genRecord(ShotReleaseSnapshot.class);
}
