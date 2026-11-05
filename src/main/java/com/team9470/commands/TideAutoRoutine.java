package com.team9470.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team9470.FieldConstants;
import com.team9470.subsystems.dye.Dye;
import com.team9470.subsystems.dye.DyeConstants;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * "Tide Auto": a single-sweep + depot auto (the same plan 4414 runs, without the hub slam), with
 * SHOOT ON THE MOVE and NO TURRET. The shooter is fixed to the robot and shoots out the back, so
 * the robot turns its back toward the Shooter's aim point to aim, while it keeps driving.
 *
 * <ol>
 *   <li>Drive out under our trench and sweep the neutral zone right along the center line with
 *       the intake open (bumpers stay 5 cm from the line, never over it).</li>
 *   <li>Come back over the bump. From the moment the whole robot is inside our alliance zone, it
 *       SHOOTS ON THE MOVE and never stops: it drives (capped at 0.8 m/s in the path) past shoot
 *       spot 1 and through the depot, intake open and front-first so it picks up the depot balls,
 *       turning so its back always points at the Shooter's aim point.</li>
 *   <li>After the depot it keeps shooting on the move (now also agitating) as it heads for our
 *       trench, until it is {@link #STOP_AIMING_BEFORE_ZONE_LINE_METERS} from the zone line. Then
 *       it stops aiming and straightens out to the path heading, so it goes under the trench
 *       straight, and ends near the middle of the field on our side.</li>
 * </ol>
 *
 * <p><b>The Shooter does the math.</b> Every loop the Shooter works out the aim point (the hub
 * shifted against the robot's motion), the flywheel speed and hood angle for that distance, and the
 * field heading the robot should have ({@link Shooter#getTargetHeading()}). This routine only:
 * turns on the Shooter's auto aim, gives the Shooter's target heading to the drivetrain while
 * aiming, and feeds when {@link Shooter#isReadyToShoot()} is true and the robot is in our zone.
 *
 * <p><b>Feeding.</b> A ball is only fed when the robot is fully inside our alliance zone AND
 * {@link Shooter#isReadyToShoot()} (flywheel at speed, hood on target, robot aimed, not faster than
 * 2.5 m/s, aim point within the shot tables). Feeding runs the Dye {@link #FEED_RATE_SCALE} times
 * faster than Dye.feed(); otherwise the Dye runs {@link Dye#reverse()}. Watch {@code Auto/InZone},
 * {@code Auto/ShooterReady} and {@code shooter/AimErrorDegrees} on the dashboard to see what it is
 * waiting for.
 *
 * <p><b>The path.</b> TideAuto.traj only has Stop Points at the start and the end. It is split
 * (without stopping) at the depot into 2 pieces, so the code knows when the depot is done. It has a
 * Max Velocity constraint of 0.8 m/s from the "zone entry" waypoint to the "depot exit" waypoint,
 * so the robot drives slowly while it shoots. If you change it in Choreo, keep the depot as the only
 * split and no Stop Points in the middle.
 *
 * <p><b>Agitating.</b> Only after the depot, while still aiming (before the depot the intake has to
 * stay down to pick up the depot balls). The intake moves up and down ({@link #AGITATE_PERIOD_SECONDS}
 * per cycle) using Intake's stow() and deploy(), because Intake.agitate() as written only raises the
 * intake. It is left down when agitating stops.
 */
public final class TideAutoRoutine {
    private TideAutoRoutine() {}

    /** Name of the .traj file in src/main/deploy/choreo (without the extension). */
    private static final String TRAJECTORY_NAME = "TideAuto";

    /**
     * Choreo assumes every path is drawn on the BLUE side and rotates it when we are on red.
     * TideAuto was drawn on the RED side, so we rotate it to blue once here; Choreo's automatic red
     * flip then turns it back into exactly what was drawn. Set to false if the path is ever redrawn
     * on the blue side. (Uses flipped(), because rotateAround() in ChoreoLib 2026.0.3 mirrors.)
     */
    private static final boolean PATH_DRAWN_ON_RED_SIDE = true;

    // ---- Shooting ----

    /**
     * Feed this many times faster than Dye.feed() (1.0 = same as Dye.feed()). 2.0 = twice as fast.
     * The rollers run at FEED_VELOCITY_RPS x FEED_MULTI x this = 2 x 20 x 2.0 = 80 rps, close to a
     * Kraken/Falcon's free speed (~100 rps), so above about 2.4 the rollers can't actually go faster.
     * If the flywheel can't recover between balls, feeding pauses a lot (see Auto/ShooterReady):
     * then back this off a little.
     */
    private static final double FEED_RATE_SCALE = 2.0;

    /** One agitation cycle: intake up for half, down for half. */
    private static final double AGITATE_PERIOD_SECONDS = 1.0;

    /**
     * How far past the alliance zone line the robot's CENTER must be before we shoot.
     * 0.41 m is half the robot with bumpers (16 in), so the whole robot is inside the zone.
     */
    private static final double ZONE_MARGIN_METERS = 0.41;

    /**
     * After the depot, stop aiming this far (robot center, beyond {@link #ZONE_MARGIN_METERS}) before
     * the zone line, so the robot has room to straighten out before it goes under the trench.
     */
    private static final double STOP_AIMING_BEFORE_ZONE_LINE_METERS = 0.6;

    /** Everything must stay ready this long before feeding starts, so a flicker does not feed a ball. */
    private static final double READY_DEBOUNCE_SECONDS = 0.1;

    public static Command build(AutoFactory factory, Swerve swerve, Intake intake, Dye dye, Shooter shooter) {
        Optional<Trajectory<SwerveSample>> loaded = Choreo.loadTrajectory(TRAJECTORY_NAME);
        if (loaded.isEmpty()) {
            DriverStation.reportError(
                    "Tide Auto: could not load " + TRAJECTORY_NAME + ".traj from deploy/choreo", false);
            return Commands.none();
        }
        if (shooter == null) {
            DriverStation.reportError("Tide Auto: needs the Shooter (it sets the flywheel speed and aim).", false);
            return Commands.none();
        }

        Trajectory<SwerveSample> path = loaded.get();
        if (PATH_DRAWN_ON_RED_SIDE) {
            path = path.flipped();
        }

        // Split (without stopping) at the depot: [0] start -> depot, [1] depot -> end.
        Optional<Trajectory<SwerveSample>> toDepot = path.getSplit(0);
        Optional<Trajectory<SwerveSample>> toEnd = path.getSplit(1);
        if (path.splits().size() != 2 || toDepot.isEmpty() || toEnd.isEmpty()) {
            DriverStation.reportError("Tide Auto: " + TRAJECTORY_NAME + ".traj must be split into exactly 2 "
                    + "pieces (one split at the depot, no Stop Point there). It has "
                    + path.splits().size() + ".", false);
            return Commands.none();
        }

        BooleanSupplier inZone = () -> inOurAllianceZone(swerve, ZONE_MARGIN_METERS);
        BooleanSupplier aimAfterDepot = () -> inOurAllianceZone(
                swerve, ZONE_MARGIN_METERS + STOP_AIMING_BEFORE_ZONE_LINE_METERS);
        DoubleSupplier aimHeading = () -> shooter.getTargetHeading().getRadians();

        return Commands.sequence(
                factory.resetOdometry(path),
                shooterAimsAtHub(shooter),
                Commands.runOnce(intake::deploy, intake),

                // 1. Sweep (intake open), back over the bump. Once fully in our zone: aim and shoot
                //    on the move, past shoot spot 1 and through the depot (intake open).
                Commands.deadline(
                        new ScaledPathFollower(swerve, toDepot.get(), 1.0, inZone, aimHeading),
                        shoot(dye, shooter, inZone)),

                // 2. Keep shooting on the move (and agitating) toward our trench, then straighten
                //    out, go under the trench and end near the middle on our side.
                Commands.deadline(
                        new ScaledPathFollower(swerve, toEnd.get(), 1.0, aimAfterDepot, aimHeading),
                        shoot(dye, shooter, aimAfterDepot),
                        agitate(intake).onlyWhile(aimAfterDepot)),
                Commands.runOnce(intake::deploy, intake),
                stop(swerve))
                .withName("Tide Auto");
    }

    /** Makes the robot hold still. The auto keeps control of the drivetrain the whole time, so we stop it ourselves. */
    private static Command stop(Swerve swerve) {
        return Commands.runOnce(() -> swerve.setChassisSpeeds(new ChassisSpeeds()), swerve);
    }

    /** Turns on the Shooter's own aiming at our hub, with shoot-on-the-move correction. */
    private static Command shooterAimsAtHub(Shooter shooter) {
        return Commands.runOnce(() -> {
            shooter.targetHub = true;
            shooter.autoAim = true;
            shooter.shootOnTheMove = true;
        });
    }

    /**
     * True when the robot's center is at least {@code margin} meters inside our own alliance zone
     * (works on red and blue). With {@link #ZONE_MARGIN_METERS} the whole robot is inside.
     */
    private static boolean inOurAllianceZone(Swerve swerve, double margin) {
        double robotX = swerve.getPose().getX();
        double zoneDepth = FieldConstants.LinesVertical.allianceZone; // blue zone line, ~4.02 m from the blue wall
        if (swerve.getAlliance() == Alliance.Red) {
            return robotX >= FieldConstants.fieldLength - zoneDepth + margin;
        }
        return robotX <= zoneDepth - margin;
    }

    /**
     * Shoots: feeds balls ({@link #FEED_RATE_SCALE} times faster than {@link Dye#feed()}) whenever we
     * are in our zone and the Shooter is ready, and holds them back with {@link Dye#reverse()} otherwise. Runs until interrupted; when it
     * ends the Dye goes back to {@link Dye#reverse()}.
     */
    private static Command shoot(Dye dye, Shooter shooter, BooleanSupplier inZone) {
        Debouncer readyFor = new Debouncer(READY_DEBOUNCE_SECONDS, DebounceType.kRising);
        return dye.startRun(
                        // Start every shooting step as "not ready" so the debounce really waits.
                        () -> readyFor.calculate(false),
                        () -> {
                            boolean zone = inZone.getAsBoolean();
                            boolean ready = shooter.isReadyToShoot();
                            SmartDashboard.putBoolean("Auto/InZone", zone);
                            SmartDashboard.putBoolean("Auto/ShooterReady", ready);
                            if (readyFor.calculate(zone && ready)) {
                                feedFast(dye);
                            } else {
                                dye.reverse();
                            }
                        })
                .finallyDo(dye::reverse);
    }

    /** Same as Dye.feed(), but {@link #FEED_RATE_SCALE} times faster (rotor and rollers). */
    private static void feedFast(Dye dye) {
        double rotor = DyeConstants.FEED_VELOCITY_RPS * FEED_RATE_SCALE;
        dye.setVelocity(rotor, rotor * DyeConstants.FEED_MULTI);
    }

    /**
     * Agitates: moves the intake up ({@link Intake#stow()}) and down ({@link Intake#deploy()})
     * every {@link #AGITATE_PERIOD_SECONDS}, starting with up. Runs until interrupted, then leaves
     * the intake open.
     */
    private static Command agitate(Intake intake) {
        Timer cycle = new Timer();
        return intake.startRun(
                        cycle::restart,
                        () -> {
                            boolean upHalf = cycle.get() % AGITATE_PERIOD_SECONDS < AGITATE_PERIOD_SECONDS / 2.0;
                            if (upHalf) {
                                intake.stow();
                            } else {
                                intake.deploy();
                            }
                        })
                .finallyDo(intake::deploy);
    }

    // ---------------------------------------------------------------------------------------
    // Path follower
    // ---------------------------------------------------------------------------------------

    /**
     * Follows one piece of a Choreo trajectory like ChoreoLib does, with two extras:
     * <ul>
     *   <li>It can play the path back slower ({@code speedScale}): same line, same headings, less speed.</li>
     *   <li>While {@code aimActive} is true, the HEADING comes from {@code aimHeading} (the Shooter's
     *       target heading) instead of the path. The robot still drives the path's line at the path's
     *       speed; only which way it faces changes, so the shooter on the back points at the aim
     *       point while it drives. This is what makes shoot on the move work without a turret.</li>
     * </ul>
     */
    private static final class ScaledPathFollower extends Command {
        private final Swerve swerve;
        private final Trajectory<SwerveSample> trajectory;
        private final double speedScale;
        private final BooleanSupplier aimActive;
        private final DoubleSupplier aimHeading;

        private double pathTime;
        private double lastTimestamp;

        /**
         * @param swerve the drivetrain
         * @param trajectory a blue-side trajectory; it is flipped for red automatically, like ChoreoLib
         * @param speedScale playback speed: 1.0 = as planned, 0.5 = half speed
         * @param aimActive while true, face {@code aimHeading} instead of the path's heading
         * @param aimHeading field heading to face (radians) while aiming
         */
        ScaledPathFollower(Swerve swerve, Trajectory<SwerveSample> trajectory, double speedScale,
                BooleanSupplier aimActive, DoubleSupplier aimHeading) {
            this.swerve = swerve;
            this.trajectory = trajectory;
            this.speedScale = Math.max(0.05, Math.min(1.0, speedScale));
            this.aimActive = aimActive;
            this.aimHeading = aimHeading;
            addRequirements(swerve);
        }

        @Override
        public void initialize() {
            pathTime = 0.0;
            lastTimestamp = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {
            double now = Timer.getFPGATimestamp();
            double dt = now - lastTimestamp;
            lastTimestamp = now;

            // Move our own "path clock" forward: at 0.7x, 20 ms of real time = 14 ms of path.
            pathTime = Math.min(pathTime + dt * speedScale, trajectory.getTotalTime());

            boolean aiming = aimActive.getAsBoolean();
            boolean flipForRed = swerve.getAlliance() == Alliance.Red;
            trajectory.sampleAt(pathTime, flipForRed).ifPresent(sample -> {
                SwerveSample s = scaled(sample, speedScale);
                if (aiming) {
                    s = withHeading(s, aimHeading.getAsDouble());
                }
                swerve.followPath(s);
            });

            SmartDashboard.putNumber("Auto/PathTime", pathTime);
            SmartDashboard.putBoolean("Auto/AimingWhileDriving", aiming);
        }

        @Override
        public boolean isFinished() {
            return pathTime >= trajectory.getTotalTime();
        }

        /**
         * Same spot on the path, played back at {@code k} times speed.
         * Speeds scale by k; accelerations and wheel forces scale by k squared.
         */
        private static SwerveSample scaled(SwerveSample s, double k) {
            double k2 = k * k;
            return new SwerveSample(
                    s.t, s.x, s.y, s.heading,
                    s.vx * k, s.vy * k, s.omega * k,
                    s.ax * k2, s.ay * k2, s.alpha * k2,
                    times(s.moduleForcesX(), k2),
                    times(s.moduleForcesY(), k2));
        }

        /**
         * Same spot and speed on the path, but facing {@code heading}. The path's turning speed is
         * dropped (the drivetrain's heading controller turns toward the new heading by itself), and
         * so are the wheel forces, because they were planned for the path's heading.
         */
        private static SwerveSample withHeading(SwerveSample s, double heading) {
            return new SwerveSample(
                    s.t, s.x, s.y, heading,
                    s.vx, s.vy, 0.0,
                    s.ax, s.ay, 0.0,
                    new double[4], new double[4]);
        }

        private static double[] times(double[] values, double k) {
            double[] out = new double[values.length];
            for (int i = 0; i < values.length; i++) {
                out[i] = values[i] * k;
            }
            return out;
        }
    }
}
