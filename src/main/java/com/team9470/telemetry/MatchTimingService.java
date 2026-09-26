package com.team9470.telemetry;

import com.team9470.telemetry.structs.PracticeTimerSnapshot;

public final class MatchTimingService {
    private static final MatchTimingService instance = new MatchTimingService();

    public static MatchTimingService getInstance() {
        return instance;
    }

    private PracticeTimerSnapshot snapshot = new PracticeTimerSnapshot(
            PracticeTimerTracker.Phase.IDLE.code(),
            false,
            false,
            false,
            false,
            0,
            0.0,
            Double.NaN,
            Double.NaN,
            0.0,
            0.0,
            0.0,
            0.0,
            Double.NaN,
            PracticeTimerTracker.START_SOURCE_NONE,
            PracticeTimerTracker.Zone.NONE.code(),
            false,
            false,
            false,
            0.0,
            0.0,
            0.0);

    private MatchTimingService() {
    }

    public synchronized void update(PracticeTimerTracker.Output output) {
        if (output == null) {
            return;
        }
        snapshot = output.snapshot();
    }

    public synchronized PracticeTimerSnapshot snapshot() {
        return snapshot;
    }

    public synchronized boolean timingKnown() {
        return snapshot.active() && snapshot.zoneKnown();
    }

    public synchronized boolean zoneActive() {
        return snapshot.zoneActive();
    }

    public synchronized boolean endgame() {
        return snapshot.endgame();
    }

    public synchronized double zoneRemainingSec() {
        return snapshot.zoneRemainingSec();
    }

    public synchronized int phaseCode() {
        return snapshot.phaseCode();
    }
}
