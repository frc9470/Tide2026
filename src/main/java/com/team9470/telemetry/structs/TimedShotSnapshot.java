package com.team9470.telemetry.structs;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;
import edu.wpi.first.util.struct.StructSerializable;

public record TimedShotSnapshot(
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
        int reasonCode,
        boolean shotIsFeedMode) implements StructSerializable {
    public static final Struct<TimedShotSnapshot> struct = StructGenerator.genRecord(TimedShotSnapshot.class);
}
