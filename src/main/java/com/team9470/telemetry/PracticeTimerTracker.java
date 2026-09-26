package com.team9470.telemetry;

import com.team9470.telemetry.structs.PracticeTimerSnapshot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import java.util.Optional;

/**
 * Tracks DS-driven practice mode phases and publishes a dashboard-friendly
 * snapshot each loop.
 */
public class PracticeTimerTracker {
    public static final int START_SOURCE_NONE = -1;
    public static final int START_SOURCE_PRACTICE_EDGE = 0;
    public static final int START_SOURCE_AUTO_EDGE_FALLBACK = 1;

    public static final double DEFAULT_PRESTART_SEC = 5.0;
    public static final double DEFAULT_AUTO_SEC = 20.0;
    public static final double DEFAULT_TRANSITION_SEC = 3.0;
    public static final double DEFAULT_TELEOP_SEC = 140.0;

    private static final double SHIFT_TRANSITION_SEC = 10.0;
    private static final double ENDGAME_WINDOW_SEC = 30.0;

    public enum Phase {
        IDLE(0, "IDLE"),
        ARMED(1, "ARMED"),
        AUTO(2, "AUTO"),
        TRANSITION(3, "TRANSITION"),
        TELEOP(4, "TELEOP"),
        ENDED(5, "ENDED"),
        ABORTED(6, "ABORTED");

        private final int code;
        private final String label;

        Phase(int code, String label) {
            this.code = code;
            this.label = label;
        }

        public int code() {
            return code;
        }

        public String label() {
            return label;
        }
    }

    public enum Zone {
        NONE(0, "NONE"),
        TRANSITION(1, "TRANSITION"),
        SHIFT1(2, "SHIFT1"),
        SHIFT2(3, "SHIFT2"),
        SHIFT3(4, "SHIFT3"),
        SHIFT4(5, "SHIFT4"),
        ENDGAME(6, "ENDGAME");

        private final int code;
        private final String label;

        Zone(int code, String label) {
            this.code = code;
            this.label = label;
        }

        public int code() {
            return code;
        }

        public String label() {
            return label;
        }
    }

    public record DriverStationSample(
            double nowSec,
            MatchType matchType,
            boolean isFMSAttached,
            boolean isDSAttached,
            boolean isAutonomousEnabled,
            boolean isTeleopEnabled,
            boolean isTestEnabled,
            boolean isDisabled,
            double matchTimeSec,
            Optional<Alliance> alliance,
            String gameSpecificMessage) {
        public DriverStationSample {
            if (alliance == null) {
                alliance = Optional.empty();
            }
            if (gameSpecificMessage == null) {
                gameSpecificMessage = "";
            }
            if (matchType == null) {
                matchType = MatchType.None;
            }
        }
    }

    public record Output(PracticeTimerSnapshot snapshot, String phaseLabel, String zoneLabel) {
    }

    private final double prestartSec;
    private final double autoSec;
    private final double transitionSec;
    private final double teleopSec;
    private final double totalDurationSec;

    private Phase phase = Phase.IDLE;
    private int runId = 0;
    private int startSourceCode = START_SOURCE_NONE;
    private boolean practiceMatchTypeDetected = false;
    private double runStartSec = Double.NaN;
    private double phaseStartSec = Double.NaN;
    private double latchedTotalElapsedSec = 0.0;
    private double latchedTotalRemainingSec = 0.0;

    private boolean lastPracticeMatchTypeSelected = false;
    private boolean lastAutonomousEnabled = false;
    private boolean lastTeleopEnabled = false;

    private boolean dsMatchTimeRejectedForPhase = false;
    private double lastDsMatchTimeSec = Double.NaN;

    private Boolean lastRedInactiveFirst = null;

    public PracticeTimerTracker() {
        this(DEFAULT_PRESTART_SEC, DEFAULT_AUTO_SEC, DEFAULT_TRANSITION_SEC, DEFAULT_TELEOP_SEC);
    }

    public PracticeTimerTracker(double prestartSec, double autoSec, double transitionSec, double teleopSec) {
        this.prestartSec = Math.max(0.0, prestartSec);
        this.autoSec = Math.max(0.0, autoSec);
        this.transitionSec = Math.max(0.0, transitionSec);
        this.teleopSec = Math.max(0.0, teleopSec);
        this.totalDurationSec = this.prestartSec + this.autoSec + this.transitionSec + this.teleopSec;
        this.latchedTotalRemainingSec = this.totalDurationSec;
    }

    public Output update(DriverStationSample sample) {
        boolean practiceMatchTypeSelected = !sample.isFMSAttached() && sample.matchType() == MatchType.Practice;
        boolean localPracticeCountdownVisible = isLocalPracticeCountdownVisible(sample);
        boolean practiceSelected = practiceMatchTypeSelected || localPracticeCountdownVisible;
        boolean practiceEdge = practiceMatchTypeSelected && !lastPracticeMatchTypeSelected;
        boolean autoEnabledEdge = sample.isAutonomousEnabled() && !lastAutonomousEnabled;
        boolean teleopEnabledEdge = sample.isTeleopEnabled() && !lastTeleopEnabled;

        if (practiceEdge && !isActivePhase(phase)) {
            startRun(sample.nowSec(), START_SOURCE_PRACTICE_EDGE);
            practiceMatchTypeDetected = true;
            if (sample.isAutonomousEnabled()) {
                enterPhase(Phase.AUTO, sample.nowSec());
            } else if (sample.isTeleopEnabled()) {
                enterPhase(Phase.TELEOP, sample.nowSec());
            } else {
                enterPhase(Phase.ARMED, sample.nowSec());
            }
        } else if (!isActivePhase(phase)
                && (sample.isFMSAttached() || practiceSelected)
                && (autoEnabledEdge || teleopEnabledEdge)) {
            startRun(sample.nowSec(), START_SOURCE_AUTO_EDGE_FALLBACK);
            if (practiceSelected) {
                practiceMatchTypeDetected = true;
            }
            if (sample.isAutonomousEnabled()) {
                enterPhase(Phase.AUTO, sample.nowSec());
            } else if (sample.isTeleopEnabled()) {
                enterPhase(Phase.TELEOP, sample.nowSec());
            } else {
                enterPhase(Phase.ARMED, sample.nowSec());
            }
        }

        if (!isRunInitialized() && phase == Phase.IDLE) {
            lastPracticeMatchTypeSelected = practiceMatchTypeSelected;
            lastAutonomousEnabled = sample.isAutonomousEnabled();
            lastTeleopEnabled = sample.isTeleopEnabled();
            return buildOutput(sample, new PhaseTiming(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sample.matchTimeSec()));
        }

        if (practiceSelected) {
            practiceMatchTypeDetected = true;
        }

        processPhaseTransitions(sample, practiceSelected);

        PhaseTiming timing = computePhaseTiming(sample);
        ZoneTiming zoneTiming = computeZoneTiming(sample, timing.teleopElapsedSec(), timing.teleopRemainingSec());

        PracticeTimerSnapshot snapshot = new PracticeTimerSnapshot(
                phase.code(),
                isActivePhase(phase),
                isTerminalPhase(phase),
                practiceMatchTypeDetected,
                timing.usingDsMatchTime(),
                runId,
                sample.nowSec(),
                runStartSec,
                phaseStartSec,
                timing.phaseElapsedSec(),
                timing.phaseRemainingSec(),
                timing.totalElapsedSec(),
                timing.totalRemainingSec(),
                timing.dsMatchTimeSec(),
                startSourceCode,
                zoneTiming.zone().code(),
                zoneTiming.active(),
                zoneTiming.known(),
                zoneTiming.endgame(),
                zoneTiming.zoneRemainingSec(),
                timing.teleopElapsedSec(),
                timing.teleopRemainingSec());

        lastPracticeMatchTypeSelected = practiceMatchTypeSelected;
        lastAutonomousEnabled = sample.isAutonomousEnabled();
        lastTeleopEnabled = sample.isTeleopEnabled();

        return new Output(snapshot, phase.label(), zoneTiming.zone().label());
    }

    private void processPhaseTransitions(DriverStationSample sample, boolean practiceSelected) {
        switch (phase) {
            case IDLE, ENDED, ABORTED -> {
            }
            case ARMED -> {
                if (sample.isTestEnabled()) {
                    abortRun(sample.nowSec());
                    break;
                }
                if (sample.isAutonomousEnabled()) {
                    enterPhase(Phase.AUTO, sample.nowSec());
                    break;
                }
                if (sample.isTeleopEnabled()) {
                    enterPhase(Phase.TELEOP, sample.nowSec());
                    break;
                }
                if (!practiceSelected && sample.isDisabled() && phaseElapsed(sample.nowSec()) > prestartSec + 0.25) {
                    resetToIdle(sample.nowSec());
                }
            }
            case AUTO -> {
                if (sample.isTestEnabled()) {
                    abortRun(sample.nowSec());
                    break;
                }
                if (!sample.isAutonomousEnabled()) {
                    if (sample.isTeleopEnabled()) {
                        enterPhase(Phase.TELEOP, sample.nowSec());
                    } else if (sample.isDisabled()) {
                        enterPhase(Phase.TRANSITION, sample.nowSec());
                    }
                }
            }
            case TRANSITION -> {
                if (sample.isTestEnabled()) {
                    abortRun(sample.nowSec());
                    break;
                }
                if (sample.isTeleopEnabled()) {
                    enterPhase(Phase.TELEOP, sample.nowSec());
                    break;
                }
                if (sample.isAutonomousEnabled()) {
                    enterPhase(Phase.AUTO, sample.nowSec());
                    break;
                }
            }
            case TELEOP -> {
                if (sample.isTestEnabled()) {
                    abortRun(sample.nowSec());
                    break;
                }
                if (sample.isAutonomousEnabled()) {
                    abortRun(sample.nowSec());
                    break;
                }
                if (!sample.isTeleopEnabled() && sample.isDisabled()) {
                    endRun(sample.nowSec());
                }
            }
        }
    }

    private PhaseTiming computePhaseTiming(DriverStationSample sample) {
        double phaseElapsedSec = 0.0;
        double phaseRemainingSec = 0.0;
        double totalElapsedSec = 0.0;
        double totalRemainingSec = totalDurationSec;
        boolean usingDsMatchTime = false;
        double teleopElapsedSec = 0.0;
        double teleopRemainingSec = teleopSec;

        switch (phase) {
            case IDLE -> {
                totalElapsedSec = 0.0;
                totalRemainingSec = totalDurationSec;
            }
            case ARMED -> {
                phaseElapsedSec = clamp(phaseElapsed(sample.nowSec()), 0.0, prestartSec);
                phaseRemainingSec = clamp(prestartSec - phaseElapsedSec, 0.0, prestartSec);
                totalElapsedSec = phaseElapsedSec;
                totalRemainingSec = clamp(totalDurationSec - totalElapsedSec, 0.0, totalDurationSec);
            }
            case AUTO -> {
                phaseElapsedSec = clamp(phaseElapsed(sample.nowSec()), 0.0, autoSec);
                phaseRemainingSec = clamp(autoSec - phaseElapsedSec, 0.0, autoSec);
                if (isValidDsMatchTime(sample.matchTimeSec(), autoSec)) {
                    usingDsMatchTime = true;
                    phaseRemainingSec = clamp(sample.matchTimeSec(), 0.0, autoSec);
                    phaseElapsedSec = clamp(autoSec - phaseRemainingSec, 0.0, autoSec);
                    lastDsMatchTimeSec = sample.matchTimeSec();
                }
                totalElapsedSec = prestartSec + phaseElapsedSec;
                totalRemainingSec = clamp(totalDurationSec - totalElapsedSec, 0.0, totalDurationSec);
            }
            case TRANSITION -> {
                phaseElapsedSec = clamp(phaseElapsed(sample.nowSec()), 0.0, transitionSec);
                phaseRemainingSec = clamp(transitionSec - phaseElapsedSec, 0.0, transitionSec);
                totalElapsedSec = prestartSec + autoSec + phaseElapsedSec;
                totalRemainingSec = clamp(totalDurationSec - totalElapsedSec, 0.0, totalDurationSec);
            }
            case TELEOP -> {
                phaseElapsedSec = clamp(phaseElapsed(sample.nowSec()), 0.0, teleopSec);
                phaseRemainingSec = clamp(teleopSec - phaseElapsedSec, 0.0, teleopSec);
                if (isValidDsMatchTime(sample.matchTimeSec(), teleopSec)) {
                    usingDsMatchTime = true;
                    phaseRemainingSec = clamp(sample.matchTimeSec(), 0.0, teleopSec);
                    phaseElapsedSec = clamp(teleopSec - phaseRemainingSec, 0.0, teleopSec);
                    lastDsMatchTimeSec = sample.matchTimeSec();
                }
                teleopElapsedSec = phaseElapsedSec;
                teleopRemainingSec = phaseRemainingSec;
                totalElapsedSec = prestartSec + autoSec + transitionSec + phaseElapsedSec;
                totalRemainingSec = clamp(totalDurationSec - totalElapsedSec, 0.0, totalDurationSec);
            }
            case ENDED, ABORTED -> {
                totalElapsedSec = latchedTotalElapsedSec;
                totalRemainingSec = latchedTotalRemainingSec;
                if (phase == Phase.ENDED) {
                    teleopElapsedSec = teleopSec;
                    teleopRemainingSec = 0.0;
                }
            }
        }

        return new PhaseTiming(
                usingDsMatchTime,
                phaseElapsedSec,
                phaseRemainingSec,
                totalElapsedSec,
                totalRemainingSec,
                teleopElapsedSec,
                teleopRemainingSec,
                sample.matchTimeSec());
    }

    private ZoneTiming computeZoneTiming(DriverStationSample sample, double teleopElapsedSec,
            double teleopRemainingSec) {
        if (phase != Phase.TELEOP && phase != Phase.ENDED) {
            return new ZoneTiming(Zone.NONE, false, false, false, 0.0);
        }

        if (phase == Phase.ENDED) {
            return new ZoneTiming(Zone.ENDGAME, true, true, true, 0.0);
        }

        double elapsed = clamp(teleopElapsedSec, 0.0, teleopSec);
        double endgameStartSec = Math.max(0.0, teleopSec - ENDGAME_WINDOW_SEC);
        double preEndgameWindowSec = Math.max(0.0, endgameStartSec);
        double shiftTransitionSec = Math.min(SHIFT_TRANSITION_SEC, preEndgameWindowSec);
        double shiftWindowSec = preEndgameWindowSec > shiftTransitionSec
                ? (preEndgameWindowSec - shiftTransitionSec) / 4.0
                : 0.0;
        double shift1EndSec = shiftTransitionSec + shiftWindowSec;
        double shift2EndSec = shift1EndSec + shiftWindowSec;
        double shift3EndSec = shift2EndSec + shiftWindowSec;
        double shift4EndSec = shift3EndSec + shiftWindowSec;

        Zone zone;
        double zoneRemaining;
        if (elapsed < shiftTransitionSec) {
            zone = Zone.TRANSITION;
            zoneRemaining = shiftTransitionSec - elapsed;
        } else if (elapsed < shift1EndSec) {
            zone = Zone.SHIFT1;
            zoneRemaining = shift1EndSec - elapsed;
        } else if (elapsed < shift2EndSec) {
            zone = Zone.SHIFT2;
            zoneRemaining = shift2EndSec - elapsed;
        } else if (elapsed < shift3EndSec) {
            zone = Zone.SHIFT3;
            zoneRemaining = shift3EndSec - elapsed;
        } else if (elapsed < shift4EndSec) {
            zone = Zone.SHIFT4;
            zoneRemaining = shift4EndSec - elapsed;
        } else {
            zone = Zone.ENDGAME;
            zoneRemaining = teleopRemainingSec;
        }
        zoneRemaining = clamp(zoneRemaining, 0.0, teleopSec);

        if (zone == Zone.TRANSITION || zone == Zone.ENDGAME) {
            return new ZoneTiming(zone, true, true, zone == Zone.ENDGAME, zoneRemaining);
        }

        Optional<Boolean> parsed = parseRedInactiveFirst(sample.gameSpecificMessage());
        parsed.ifPresent(value -> lastRedInactiveFirst = value);

        if (sample.alliance().isEmpty()) {
            return new ZoneTiming(zone, false, false, false, zoneRemaining);
        }
        if (lastRedInactiveFirst == null) {
            return new ZoneTiming(zone, true, false, false, zoneRemaining);
        }

        boolean shift1Active = switch (sample.alliance().get()) {
            case Red -> !lastRedInactiveFirst;
            case Blue -> lastRedInactiveFirst;
        };
        boolean zoneActive = switch (zone) {
            case SHIFT1, SHIFT3 -> shift1Active;
            case SHIFT2, SHIFT4 -> !shift1Active;
            default -> true;
        };
        return new ZoneTiming(zone, zoneActive, true, false, zoneRemaining);
    }

    private Optional<Boolean> parseRedInactiveFirst(String gameData) {
        if (gameData == null || gameData.isEmpty()) {
            return Optional.empty();
        }
        return switch (gameData.charAt(0)) {
            case 'R' -> Optional.of(true);
            case 'B' -> Optional.of(false);
            default -> Optional.empty();
        };
    }

    private boolean isValidDsMatchTime(double matchTimeSec, double phaseDurationSec) {
        if (dsMatchTimeRejectedForPhase || !Double.isFinite(matchTimeSec)) {
            return false;
        }
        if (matchTimeSec < 0.0 || matchTimeSec > phaseDurationSec + 1.0) {
            return false;
        }
        if (Double.isFinite(lastDsMatchTimeSec) && matchTimeSec > lastDsMatchTimeSec + 0.05) {
            dsMatchTimeRejectedForPhase = true;
            return false;
        }
        return true;
    }

    private boolean isLocalPracticeCountdownVisible(DriverStationSample sample) {
        return sample.isDSAttached()
                && !sample.isFMSAttached()
                && Double.isFinite(sample.matchTimeSec())
                && sample.matchTimeSec() > 0.0;
    }

    private void startRun(double nowSec, int sourceCode) {
        runId++;
        startSourceCode = sourceCode;
        practiceMatchTypeDetected = false;
        runStartSec = nowSec;
        phaseStartSec = nowSec;
        latchedTotalElapsedSec = 0.0;
        latchedTotalRemainingSec = totalDurationSec;
        lastRedInactiveFirst = null;
        enterPhase(Phase.ARMED, nowSec);
    }

    private void abortRun(double nowSec) {
        double totalElapsed = computeLocalTotalElapsed(nowSec);
        enterPhase(Phase.ABORTED, nowSec);
        latchedTotalElapsedSec = clamp(totalElapsed, 0.0, totalDurationSec);
        latchedTotalRemainingSec = clamp(totalDurationSec - latchedTotalElapsedSec, 0.0, totalDurationSec);
    }

    private void endRun(double nowSec) {
        enterPhase(Phase.ENDED, nowSec);
        latchedTotalElapsedSec = totalDurationSec;
        latchedTotalRemainingSec = 0.0;
    }

    private void resetToIdle(double nowSec) {
        enterPhase(Phase.IDLE, nowSec);
        startSourceCode = START_SOURCE_NONE;
        practiceMatchTypeDetected = false;
        runStartSec = Double.NaN;
        phaseStartSec = Double.NaN;
        latchedTotalElapsedSec = 0.0;
        latchedTotalRemainingSec = totalDurationSec;
        lastRedInactiveFirst = null;
    }

    private void enterPhase(Phase next, double nowSec) {
        if (phase != next) {
            phase = next;
            phaseStartSec = nowSec;
            dsMatchTimeRejectedForPhase = false;
            lastDsMatchTimeSec = Double.NaN;
        }
    }

    private double computeLocalTotalElapsed(double nowSec) {
        return switch (phase) {
            case IDLE -> 0.0;
            case ARMED -> clamp(phaseElapsed(nowSec), 0.0, prestartSec);
            case AUTO -> prestartSec + clamp(phaseElapsed(nowSec), 0.0, autoSec);
            case TRANSITION -> prestartSec + autoSec + clamp(phaseElapsed(nowSec), 0.0, transitionSec);
            case TELEOP -> prestartSec + autoSec + transitionSec + clamp(phaseElapsed(nowSec), 0.0, teleopSec);
            case ENDED, ABORTED -> latchedTotalElapsedSec;
        };
    }

    private double phaseElapsed(double nowSec) {
        if (!Double.isFinite(phaseStartSec)) {
            return 0.0;
        }
        return Math.max(0.0, nowSec - phaseStartSec);
    }

    private boolean isRunInitialized() {
        return runId > 0 || isActivePhase(phase) || isTerminalPhase(phase);
    }

    private static boolean isActivePhase(Phase phase) {
        return phase == Phase.ARMED || phase == Phase.AUTO || phase == Phase.TRANSITION || phase == Phase.TELEOP;
    }

    private static boolean isTerminalPhase(Phase phase) {
        return phase == Phase.ENDED || phase == Phase.ABORTED;
    }

    private static double clamp(double value, double min, double max) {
        if (!Double.isFinite(value)) {
            return min;
        }
        return Math.max(min, Math.min(max, value));
    }

    private Output buildOutput(DriverStationSample sample, PhaseTiming timing) {
        ZoneTiming zoneTiming = new ZoneTiming(Zone.NONE, false, false, false, 0.0);
        PracticeTimerSnapshot snapshot = new PracticeTimerSnapshot(
                phase.code(),
                false,
                false,
                false,
                false,
                runId,
                sample.nowSec(),
                runStartSec,
                phaseStartSec,
                timing.phaseElapsedSec(),
                timing.phaseRemainingSec(),
                timing.totalElapsedSec(),
                timing.totalRemainingSec(),
                sample.matchTimeSec(),
                startSourceCode,
                zoneTiming.zone().code(),
                zoneTiming.active(),
                zoneTiming.known(),
                zoneTiming.endgame(),
                zoneTiming.zoneRemainingSec(),
                timing.teleopElapsedSec(),
                timing.teleopRemainingSec());
        return new Output(snapshot, phase.label(), zoneTiming.zone().label());
    }

    private record PhaseTiming(
            boolean usingDsMatchTime,
            double phaseElapsedSec,
            double phaseRemainingSec,
            double totalElapsedSec,
            double totalRemainingSec,
            double teleopElapsedSec,
            double teleopRemainingSec,
            double dsMatchTimeSec) {
    }

    private record ZoneTiming(
            Zone zone,
            boolean active,
            boolean known,
            boolean endgame,
            double zoneRemainingSec) {
    }
}
