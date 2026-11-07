package com.team9470.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team9470.FieldConstants;
import com.team9470.subsystems.dye.Dye;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.shooter.Shooter;
import com.team9470.subsystems.swerve.Swerve;
import com.team9470.subsystems.turret.Turret;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;

/**
 * "Tide Auto": a single-sweep + depot auto (the same plan 4414 runs, without the hub slam),
 * and the robot only shoots while standing still.
 *
 * <ol>
 *   <li>Drive out under our trench and sweep the neutral zone in a straight line right along the
 *       center line with the intake open: the bumpers stay 5 cm from the line, never over it.
 *       The robot keeps its heading fixed while it is near the line and only turns after backing
 *       away from it, because a turning robot's corners swing out farther than its sides.</li>
 *   <li>Come back over the bump next to the trench we started under (staying clear of the hub)
 *       and stop just inside our alliance zone. Shoot for {@link #SHOOT_AFTER_BUMP_SECONDS}
 *       while agitating.</li>
 *   <li>Intake open, drive into the depot to pick up the depot balls. Stop there and shoot for
 *       {@link #SHOOT_AT_DEPOT_SECONDS} while agitating.</li>
 *   <li>Go: drive under our trench and end near the middle of the field, on our side.</li>
 * </ol>
 *
 * <p><b>The path.</b> TideAuto.traj has real Stop Points at both shooting spots, and is split
 * there into 3 pieces (Choreo "splits"): start to shoot spot 1, shoot spot 1 to the depot, and
 * the depot to the end. This routine drives the pieces in order and shoots between them. If the
 * path is changed in Choreo, keep exactly those 2 split waypoints (each with a Stop Point).
 *
 * <p><b>Agitating.</b> While shooting, the intake moves up and down
 * ({@link #AGITATE_PERIOD_SECONDS} per cycle) to shake balls loose toward the Dye, with the
 * intake rollers and kicker running. This uses Intake's stow() and deploy() directly, because
 * Intake.agitate() as written only raises the intake (its timer is never started). After
 * shooting, the intake is left open.
 *
 * <p><b>Shooter and turret aiming ({@link #SUBSYSTEMS_AIM_THEMSELVES}).</b>
 * <ul>
 *   <li>{@code true} (default): the Shooter and Turret aim themselves. Shooter.periodic() sets the
 *       flywheel speed and hood angle from the distance to the hub every loop
 *       (setTargetVelocity(calculateVelocity()) / setTargetAngle(calculateHoodAngle())), and
 *       Turret.periodic() points the turret at the hub every loop (calculateTargetMain()). This
 *       routine does not touch their setpoints; it only makes sure both are set to target the hub
 *       (not passing), waits until they are on target, and feeds. Those lines must be uncommented
 *       in Shooter and Turret for this to shoot: until then the flywheel target stays 0 and the
 *       auto drives the whole path but never feeds.</li>
 *   <li>{@code false}: the old way, for testing before that code works. This routine sets fixed
 *       flywheel/hood values per spot (the SHOT_* constants) and points the turret itself.</li>
 * </ul>
 * If the turret is null, the auto still runs and just doesn't wait for the turret.
 *
 * <p><b>Feeding.</b> A ball is only fed when the shot is READY: flywheel at speed, hood on target,
 * and turret on target (if there is one). Feeding is {@link Dye#feed()}; when not feeding the Dye
 * runs {@link Dye#reverse()}, the same as teleop without the right trigger. Watch
 * {@code Auto/ShooterReady}, {@code Auto/HoodReady} and {@code Auto/TurretReady} on the dashboard
 * to see what it is waiting for.
 */
public final class TideAutoRoutine {
    private TideAutoRoutine() {}

    /** Name of the .traj file in src/main/deploy/choreo (without the extension). */
    private static final String TRAJECTORY_NAME = "TideAuto";

    /**
     * Choreo assumes every path is drawn on the BLUE side and rotates it automatically
     * when we are on red. TideAuto was drawn on the RED side (it ends at the red depot),
     * so we rotate it to the blue side once here. Choreo's automatic red flip then turns
     * it back into exactly what was drawn.
     *
     * <p>Set this to false if the path is ever redrawn on the blue side (recommended).
     *
     * <p>Note: this uses Trajectory.flipped(), not rotateAround(). In ChoreoLib 2026.0.3,
     * Trajectory.rotateAround() mirrors left/right instead of rotating.
     */
    private static final boolean PATH_DRAWN_ON_RED_SIDE = true;

    /**
     * true: Shooter.periodic() and Turret.periodic() calculate their own targets from the robot's
     * position (their aiming code uncommented and working), so this routine only waits for them
     * and feeds. false: this routine uses the fixed SHOT_* values and aims the turret itself.
     */
    private static final boolean SUBSYSTEMS_AIM_THEMSELVES = true;

    // ---- How long to shoot at each spot ----

    /**
     * Seconds to stand still and shoot after the bump. This is the stop with the most balls
     * (everything from the neutral zone sweep), so it gets the longest time.
     * With 5.0 here and 3.0 at the depot, the whole auto ends at about 17.5 s of 20.
     */
    private static final double SHOOT_AFTER_BUMP_SECONDS = 5.0;

    /** Seconds to stand still and shoot at the depot (the intake keeps running). */
    private static final double SHOOT_AT_DEPOT_SECONDS = 3.0;

    /**
     * One agitation cycle while shooting: intake up for half, down for half.
     * 1.0 s matches the cycle written in Intake.periodic().
     */
    private static final double AGITATE_PERIOD_SECONDS = 1.0;

    // ---- Driving ----

    /**
     * Drive into the depot at this fraction of the planned speed so the intake can grab balls.
     * 0.7 adds about 0.6 s. Every 0.1 lower adds roughly another 0.3 s, so watch the 20 s limit.
     */
    private static final double DEPOT_DRIVE_SPEED_SCALE = 0.7;

    /**
     * How far past the alliance zone line the robot's CENTER must be before we shoot.
     * 0.41 m is half the robot with bumpers (16 in), so the whole robot is inside the zone.
     * Both shooting spots are well inside the zone; this is just a safety check.
     */
    private static final double ZONE_MARGIN_METERS = 0.41;

    // ---- Fixed shooter setpoints per spot: ONLY used when SUBSYSTEMS_AIM_THEMSELVES is false ----

    /** Flywheel speed (Shooter.targetShooterVelocity units) for shoot spot 1, about 2.3 m from the hub. */
    private static final double SHOT_1_FLYWHEEL = 50.0;

    /** Hood angle in degrees for shoot spot 1. Hood range is 0 to 50 degrees (ShooterConstants). */
    private static final double SHOT_1_HOOD_DEGREES = 20.0;

    /** Flywheel speed for shoot spot 2 at the depot, about 4.5 m from the hub (farther = faster). */
    private static final double SHOT_2_FLYWHEEL = 60.0;

    /** Hood angle in degrees for shoot spot 2. */
    private static final double SHOT_2_HOOD_DEGREES = 30.0;

    /** Hood angle to leave the hood at after the last shot. */
    private static final double HOOD_REST_DEGREES = 0.0;

    // ---- Turret aiming (only used if a turret exists) ----

    /**
     * Added to the aim angle, in degrees. 0 means turret angle 0 points straight out the front of
     * the robot and positive angles turn counter-clockwise (the same convention as the robot's
     * heading). If your turret's zero points somewhere else, put that offset here.
     */
    private static final double TURRET_ZERO_OFFSET_DEGREES = 0.0;

    // ---- Shot readiness (tune on the real robot) ----

    /** Flywheel must be within this many rotations per second of its target to feed. */
    private static final double SHOOTER_SPEED_TOLERANCE_RPS = 2.0;

    /** Flywheel target must be at least this fast, so we never feed into a stopped shooter. */
    private static final double SHOOTER_MIN_TARGET_RPS = 1.0;

    /** Hood must be within this many degrees of its target angle to feed. */
    private static final double HOOD_TOLERANCE_DEGREES = 1.5;

    /** Turret must be within this many degrees of its target angle to feed. */
    private static final double TURRET_TOLERANCE_DEGREES = 3.0;

    /** Everything must stay ready this long before feeding starts, so a flicker does not feed a ball. */
    private static final double READY_DEBOUNCE_SECONDS = 0.1;

    public static Command build(
            AutoFactory factory, Swerve swerve, Intake intake, Dye dye, Shooter shooter, Turret turret) {
        Optional<Trajectory<SwerveSample>> loaded = Choreo.loadTrajectory(TRAJECTORY_NAME);
        if (loaded.isEmpty()) {
            DriverStation.reportError(
                    "Tide Auto: could not load " + TRAJECTORY_NAME + ".traj from deploy/choreo", false);
            return Commands.none();
        }
        if (shooter == null) {
            DriverStation.reportWarning(
                    "Tide Auto: the shooter is not created, so the auto cannot spin it up or check if "
                            + "shots are ready. It will feed for the full shooting time at each stop.", false);
        }
        if (turret == null) {
            DriverStation.reportWarning("Tide Auto: no turret, so the auto will not aim.", false);
        }

        Trajectory<SwerveSample> path = loaded.get();
        if (PATH_DRAWN_ON_RED_SIDE) {
            path = path.flipped();
        }

        // The path is split at the two shooting spots: [0] start -> shoot spot 1,
        // [1] shoot spot 1 -> depot, [2] depot -> end.
        Optional<Trajectory<SwerveSample>> toShootSpot = path.getSplit(0);
        Optional<Trajectory<SwerveSample>> toDepot = path.getSplit(1);
        Optional<Trajectory<SwerveSample>> toEnd = path.getSplit(2);
        if (path.splits().size() != 3 || toShootSpot.isEmpty() || toDepot.isEmpty() || toEnd.isEmpty()) {
            DriverStation.reportError("Tide Auto: " + TRAJECTORY_NAME + ".traj must be split into exactly 3 "
                    + "pieces (split + Stop Point at shoot spot 1 and at the depot). It has "
                    + path.splits().size() + ".", false);
            return Commands.none();
        }

        return Commands.sequence(
                factory.resetOdometry(path),
                aimAtHub(shooter, turret),

                // 1. Sweep the neutral zone (intake open), back over the bump, stop in our zone.
                //    The flywheel spins up for shoot spot 1 while we drive.
                Commands.runOnce(intake::deploy, intake),
                setShooter(shooter, SHOT_1_FLYWHEEL, SHOT_1_HOOD_DEGREES),
                new ScaledPathFollower(swerve, toShootSpot.get(), 1.0),
                holdStill(swerve),

                // 2. Shoot standing still, agitating the intake to push balls toward the Dye.
                shootFor(SHOOT_AFTER_BUMP_SECONDS, swerve, intake, dye, shooter, turret),

                // 3. Intake open, drive into the depot (flywheel/hood change for spot 2 on the way),
                //    stop and shoot (agitating again).
                Commands.runOnce(intake::deploy, intake),
                setShooter(shooter, SHOT_2_FLYWHEEL, SHOT_2_HOOD_DEGREES),
                new ScaledPathFollower(swerve, toDepot.get(), DEPOT_DRIVE_SPEED_SCALE),
                holdStill(swerve),
                shootFor(SHOOT_AT_DEPOT_SECONDS, swerve, intake, dye, shooter, turret),

                // 4. Go: under our trench, end near the middle on our side (with fixed setpoints,
                //    the flywheel is turned off here; when the Shooter aims itself it keeps running).
                setShooter(shooter, 0.0, HOOD_REST_DEGREES),
                new ScaledPathFollower(swerve, toEnd.get(), 1.0),
                stop(swerve))
                .withName("Tide Auto");
    }

    /**
     * Stand still and shoot for {@code seconds} while agitating. Only feeds when the shot is
     * ready, and only if we really are inside our alliance zone (otherwise it skips the shot
     * and warns).
     */
    private static Command shootFor(
            double seconds, Swerve swerve, Intake intake, Dye dye, Shooter shooter, Turret turret) {
        return Commands.either(
                Commands.parallel(shoot(dye, shooter, turret), agitate(intake), aimTurret(swerve, turret))
                        .withTimeout(seconds),
                Commands.runOnce(() -> DriverStation.reportWarning(
                        "Tide Auto: stopped outside our alliance zone, skipping this shot", false)),
                () -> inOurAllianceZone(swerve));
    }

    /**
     * Makes sure the Shooter and Turret target the hub (not passing) for the whole auto. Only
     * matters when they aim themselves.
     */
    private static Command aimAtHub(Shooter shooter, Turret turret) {
        return Commands.runOnce(() -> {
            if (shooter != null) {
                shooter.targetHub = true;
            }
            if (turret != null) {
                turret.targetHub = true;
            }
        });
    }

    /**
     * Sets the flywheel speed and hood angle that Shooter.periodic() runs to. Does nothing when
     * the Shooter calculates its own setpoints (it would overwrite these every loop anyway).
     */
    private static Command setShooter(Shooter shooter, double flywheel, double hoodDegrees) {
        if (shooter == null || SUBSYSTEMS_AIM_THEMSELVES) {
            return Commands.none();
        }
        return Commands.runOnce(() -> {
            shooter.setTargetVelocity(flywheel);
            //shooter.setTargetAngle(hoodDegrees);
        });
    }

    /**
     * Keeps the turret pointed at OUR hub, from where the robot actually is. Does nothing if
     * there is no turret. The angle is relative to the front of the robot, counter-clockwise
     * positive, wrapped to -180..180 degrees, plus {@link #TURRET_ZERO_OFFSET_DEGREES}.
     */
    private static Command aimTurret(Swerve swerve, Turret turret) {
        if (turret == null || SUBSYSTEMS_AIM_THEMSELVES) { // Turret.periodic() aims itself
            return Commands.none();
        }
        return Commands.run(() -> {
            Pose2d robot = swerve.getPose();
            Translation2d toHub = ourHub(swerve).minus(robot.getTranslation());
            double degrees = toHub.getAngle().minus(robot.getRotation()).getDegrees()
                    + TURRET_ZERO_OFFSET_DEGREES;
            Turret.targetAngle = Degrees.of(Math.IEEEremainder(degrees, 360.0));
        });
    }

    /** Our alliance's hub. FieldConstants.Hub only has the blue one, so rotate it for red. */
    private static Translation2d ourHub(Swerve swerve) {
        Translation2d blueHub = new Translation2d(
                FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY());
        if (swerve.getAlliance() == Alliance.Red) {
            return new Translation2d(
                    FieldConstants.fieldLength - blueHub.getX(), FieldConstants.fieldWidth - blueHub.getY());
        }
        return blueHub;
    }

    /** True when the robot is fully inside our own alliance zone (works on red and blue). */
    private static boolean inOurAllianceZone(Swerve swerve) {
        double robotX = swerve.getPose().getX();
        double zoneDepth = FieldConstants.LinesVertical.allianceZone; // blue zone line, ~4.02 m from the blue wall
        if (swerve.getAlliance() == Alliance.Red) {
            return robotX >= FieldConstants.fieldLength - zoneDepth + ZONE_MARGIN_METERS;
        }
        return robotX <= zoneDepth - ZONE_MARGIN_METERS;
    }

    /**
     * Shoots: feeds balls with {@link Dye#feed()} whenever the shot is ready, and holds them back
     * with {@link Dye#reverse()} whenever it is not. Runs until interrupted; when it ends the Dye
     * goes back to {@link Dye#reverse()}.
     */
    private static Command shoot(Dye dye, Shooter shooter, Turret turret) {
        Debouncer readyFor = new Debouncer(READY_DEBOUNCE_SECONDS, DebounceType.kRising);
        return dye.startRun(
                        // Start every shooting step as "not ready" so the debounce really waits.
                        () -> readyFor.calculate(false),
                        () -> {
                            // if (readyFor.calculate(isShotReady(shooter, turret))) {
                            //     dye.feed();
                            // } else {
                            //     dye.reverse();
                            // }
                        })
                .finallyDo(dye::reverse);
    }

    /**
     * Agitates: moves the intake up ({@link Intake#stow()}) and down ({@link Intake#deploy()})
     * every {@link #AGITATE_PERIOD_SECONDS}, starting with up. The intake rollers and kicker keep
     * running the whole time (Intake runs them whenever it has been deployed). Runs until
     * interrupted, then leaves the intake open.
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

    /**
     * True when the flywheel is at speed and the hood (and turret, if there is one) are on target.
     * Also publishes each part to the dashboard so you can see what the auto is waiting for.
     */
    // private static boolean isShotReady(Shooter shooter, Turret turret) {
    //     if (shooter == null) {
    //         return true; // can't check; warned about this when the auto was built
    //     }
    //     double targetRps = Shooter.targetShooterVelocity;
    //     boolean shooterReady = targetRps >= SHOOTER_MIN_TARGET_RPS
    //             && Math.abs(shooter.getVelocity() - targetRps) <= SHOOTER_SPEED_TOLERANCE_RPS;
    //     //boolean hoodReady = Math.abs(
    //             //shooter.getAngle().in(Degrees) - Shooter.targetHoodAngle) <= HOOD_TOLERANCE_DEGREES;
    //     boolean turretReady = turret == null // no turret: nothing to wait for
    //             || Math.abs(Turret.getAngle().in(Degrees) - Turret.targetAngle.in(Degrees)) <= TURRET_TOLERANCE_DEGREES;

    //     SmartDashboard.putBoolean("Auto/ShooterReady", shooterReady);
    //     //SmartDashboard.putBoolean("Auto/HoodReady", hoodReady);
    //     SmartDashboard.putBoolean("Auto/TurretReady", turretReady);
    //     //return shooterReady && hoodReady && turretReady;
    // }

    /**
     * Stop and X-lock the wheels while shooting, so a bump from another robot doesn't push us
     * off the spot (and the turret's aim stays good). The next path piece un-locks them.
     */
    private static Command holdStill(Swerve swerve) {
        return Commands.runOnce(swerve::stopWithXLock, swerve);
    }

    /** Makes the robot hold still. The auto keeps control of the drivetrain the whole time, so we stop it ourselves. */
    private static Command stop(Swerve swerve) {
        return Commands.runOnce(() -> swerve.setChassisSpeeds(new ChassisSpeeds()), swerve);
    }

    // ---------------------------------------------------------------------------------------
    // Scaled path follower
    // ---------------------------------------------------------------------------------------

    /**
     * Follows one piece of a Choreo trajectory like ChoreoLib does, but can play it back slower.
     *
     * <p>Think of the trajectory as a video of the robot driving. At {@code speedScale} 1.0 it
     * plays normally. At 0.7 the robot drives the exact same line on the field, with the same
     * headings, just at 70% speed. Every piece of TideAuto starts and ends with a Stop Point, so
     * the robot always starts from and ends at a standstill.
     *
     * <p>Going slower than the path was planned for is always safe for the drivetrain: every
     * speed and acceleration in the sample is scaled down, never up.
     */
    private static final class ScaledPathFollower extends Command {
        private final Swerve swerve;
        private final Trajectory<SwerveSample> trajectory;
        private final double speedScale;

        private double pathTime;
        private double lastTimestamp;

        /**
         * @param swerve the drivetrain
         * @param trajectory a blue-side trajectory; it is flipped for red automatically, like ChoreoLib
         * @param speedScale playback speed: 1.0 = as planned, 0.5 = half speed
         */
        ScaledPathFollower(Swerve swerve, Trajectory<SwerveSample> trajectory, double speedScale) {
            this.swerve = swerve;
            this.trajectory = trajectory;
            this.speedScale = Math.max(0.05, Math.min(1.0, speedScale));
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

            // Look up that point on the path (flipped for red, like ChoreoLib does), scale its
            // speeds, and hand it to the drivetrain's normal path follower.
            boolean flipForRed = swerve.getAlliance() == Alliance.Red;
            trajectory.sampleAt(pathTime, flipForRed)
                    .ifPresent(sample -> swerve.followPath(scaled(sample, speedScale)));

            SmartDashboard.putNumber("Auto/PathTime", pathTime);
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

        private static double[] times(double[] values, double k) {
            double[] out = new double[values.length];
            for (int i = 0; i < values.length; i++) {
                out[i] = values[i] * k;
            }
            return out;
        }
    }
}
