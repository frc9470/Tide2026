package com.team9470.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.team9470.subsystems.dye.Dye;
import com.team9470.subsystems.intake.Intake;
import com.team9470.subsystems.swerve.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * "Tide Auto": drive the TideAuto Choreo path with the intake down, and feed balls
 * into the shooter with the Dye.
 *
 * <p>The turret and shooter aim themselves in their own periodic() methods, so this
 * routine never commands them. A "shot" here just means running {@link Dye#feed()}.
 *
 * <p>Two modes, picked automatically from the .traj file:
 * <ul>
 *   <li><b>No splits in the path</b> (the current file): the path runs start to finish
 *       without stopping. The first shot happens while driving, at
 *       {@link #MOVING_SHOT_START_SEC}. The second shot happens after the path ends.</li>
 *   <li><b>Path has splits</b>: the robot stops at every split, shoots, then drives the
 *       next segment. To use this, open TideAuto in Choreo, tick "Split" on the waypoint
 *       you want to shoot from, ALSO add a Stop Point constraint on that same waypoint
 *       (otherwise the robot brakes hard from full speed), and regenerate.</li>
 * </ul>
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

    /** Seconds into the path to start the moving shot (no-split mode). ~2.3 s matches the original plan. */
    private static final double MOVING_SHOT_START_SEC = 2.3;

    /** How long the Dye feeds for each shot. */
    private static final double SHOT_DURATION_SEC = 1.5;

    public static Command build(AutoFactory factory, Swerve swerve, Intake intake, Dye dye) {
        Optional<Trajectory<SwerveSample>> loaded = Choreo.loadTrajectory(TRAJECTORY_NAME);
        if (loaded.isEmpty()) {
            DriverStation.reportError(
                    "Tide Auto: could not load " + TRAJECTORY_NAME + ".traj from deploy/choreo", false);
            return Commands.none();
        }

        Trajectory<SwerveSample> path = loaded.get();
        if (PATH_DRAWN_ON_RED_SIDE) {
            path = path.flipped();
        }

        // splits() always contains 0 (the start), so 2+ entries means the path really is split.
        boolean pathHasSplits = path.splits().size() >= 2;
        Command routine = pathHasSplits
                ? stopAndShoot(factory, swerve, intake, dye, path)
                : shootOnTheMove(factory, swerve, intake, dye, path);
        return routine.withName("Tide Auto");
    }

    /** Current path (no splits): drive the whole path without stopping. Shoot once while moving, once at the end. */
    private static Command shootOnTheMove(
            AutoFactory factory, Swerve swerve, Intake intake, Dye dye, Trajectory<SwerveSample> path) {
        return Commands.sequence(
                factory.resetOdometry(path),
                Commands.runOnce(intake::deploy, intake),

                // The path decides when this step ends. The moving shot runs alongside it
                // and is cut off if the path happens to finish first.
                Commands.deadline(
                        factory.trajectoryCmd(path),
                        Commands.sequence(
                                Commands.waitSeconds(MOVING_SHOT_START_SEC),
                                shoot(dye))),

                stop(swerve),
                shoot(dye),
                Commands.runOnce(intake::stow, intake));
    }

    /** Path with splits: drive each segment, stop, shoot. */
    private static Command stopAndShoot(
            AutoFactory factory, Swerve swerve, Intake intake, Dye dye, Trajectory<SwerveSample> path) {
        List<Command> steps = new ArrayList<>();
        steps.add(factory.resetOdometry(path));
        steps.add(Commands.runOnce(intake::deploy, intake));

        for (int i = 0; i < path.splits().size(); i++) {
            Optional<Trajectory<SwerveSample>> segment = path.getSplit(i);
            if (segment.isEmpty() || segment.get().samples().isEmpty()) {
                continue;
            }
            steps.add(factory.trajectoryCmd(segment.get()));
            steps.add(stop(swerve));
            steps.add(shoot(dye));
        }

        steps.add(Commands.runOnce(intake::stow, intake));
        return Commands.sequence(steps.toArray(Command[]::new));
    }

    /** Feeds balls into the shooter for a fixed time. Afterwards the Dye goes back to its default (stopped) command. */
    private static Command shoot(Dye dye) {
        return Commands.none();
        //return dye.feed().withTimeout(SHOT_DURATION_SEC);
    }

    /** Makes the robot hold still. The auto keeps control of the drivetrain the whole time, so we stop it ourselves. */
    private static Command stop(Swerve swerve) {
        return Commands.runOnce(() -> swerve.setChassisSpeeds(new ChassisSpeeds()), swerve);
    }
}