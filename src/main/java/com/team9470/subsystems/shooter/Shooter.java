package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.FieldConstants;
import com.team9470.Ports;
import com.team9470.subsystems.swerve.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * Shooter (4 flywheel motors + hood, NO turret) with SHOOT ON THE MOVE.
 *
 * <p><b>The idea.</b> A ball leaves the shooter with the robot's own velocity added to it. If the
 * robot drives at 2 m/s and the ball is in the air for 1 s, the ball lands 2 m off to the side
 * of where we aimed. So instead of aiming at the hub, we aim at a point shifted the OTHER way by
 * (robot velocity x time of flight). That point is {@link #aimPoint}. The robot's motion then
 * carries the ball onto the real hub.
 *
 * <p>The time of flight depends on how far away the aim point is, and the aim point depends on
 * the time of flight, so we go around that loop a few times ({@link #AIM_ITERATIONS}). It settles
 * after 2 or 3 rounds.
 *
 * <p><b>Every loop (periodic)</b> this class:
 * <ol>
 *   <li>Gets the robot's pose and its FIELD-relative velocity.</li>
 *   <li>Predicts where the robot will be when the ball actually leaves ({@link #RELEASE_DELAY_SECONDS}).</li>
 *   <li>Finds the aim point (our hub, or the nearest pass target when {@link #targetHub} is false),
 *       shifted against the robot's motion.</li>
 *   <li>Looks up flywheel speed and hood angle for the distance to the aim point (shot tables below).</li>
 *   <li>Runs the flywheel and hood to those targets (when {@link #autoAim} is true).</li>
 * </ol>
 *
 * <p><b>No turret: the ROBOT aims.</b> The shooter is fixed to the robot and shoots out the back
 * ({@link #SHOOTER_FACING_DEGREES}). So the drivetrain has to turn the robot until the shooter side
 * points at {@link #aimPoint}. {@link #getTargetHeading()} is the field heading the robot should
 * have for that, and {@link #getAimErrorRadians()} is how far off it is right now. Feed this to the
 * drivetrain's heading controller (the auto does this while it drives). On the move the target
 * heading keeps changing, so the robot keeps turning as it drives past the hub.
 *
 * <p><b>Feeding.</b> Only feed balls when {@link #isReadyToShoot()} is true: flywheel at speed,
 * hood on target, robot pointed at the aim point, robot not driving too fast, and the aim point in
 * the range the tables cover.
 *
 * <p><b>Works on red and blue.</b> FieldConstants only has blue-side positions, so red positions are
 * the blue ones rotated around the field center.
 *
 * <p><b>TUNE BEFORE USING.</b> All numbers in the shot tables are starting guesses. See the
 * comment above the tables for how to measure the real ones.
 */
public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final TalonFX shooterMotor3;
    private final TalonFX shooterMotor4;
    private final TalonFX hoodMotor;

    private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage hoodMotionMagic = new MotionMagicVoltage(0);
    public static double targetShooterVelocity = 0;
    public static Angle targetHoodAngle = Units.Degrees.of(0);

    /**
     * Where to aim, in field coordinates (meters): the target shifted against the robot's motion.
     * When the robot is standing still this is exactly the target. The shooter side of the robot should point here.
     */
    public static Translation2d aimPoint = new Translation2d();

    /** Field heading the robot should have so the shooter points at {@link #aimPoint}. */
    private static Rotation2d targetHeading = new Rotation2d();

    /** targetHeading minus the robot's current heading, -pi..pi radians (positive = turn CCW). */
    private static double aimErrorRadians = 0.0;

    private final Swerve poseSlave;

    /** true = shoot into our hub, false = pass to the nearest of our depot / outpost. */
    public boolean targetHub = true;

    /**
     * true = this class sets the flywheel speed and hood angle itself every loop (from the shot
     * tables). false = it only calculates the aim; you set the speed/angle with
     * setTargetVelocity() / setTargetAngle() yourself (useful for testing fixed shots).
     */
    public boolean autoAim = true;

    /**
     * true = correct the aim for the robot's motion (shoot on the move). false = aim straight at
     * the target (only right when the robot is standing still).
     */
    public boolean shootOnTheMove = true;

    // ---- Where the shooter is on the robot (no turret) ----

    /**
     * Direction the shooter shoots, relative to the FRONT of the robot (the intake side), counter-
     * clockwise positive. 180 = out the back, 0 = out the front, 90 = left, -90 = right.
     */
    public static final double SHOOTER_FACING_DEGREES = 180.0;

    /**
     * Where the ball leaves the robot, relative to the robot's center, in meters (x = forward,
     * y = left). Out the back about 0.25 m would be (-0.25, 0). Measure it on your robot. With the
     * shooter off-center, turning the robot also moves the shooter, and the math accounts for it.
     */
    private static final Translation2d SHOOTER_OFFSET_METERS = new Translation2d(0.0, 0.0);

    /**
     * The robot counts as aimed when the shooter points within this many degrees of the aim point.
     * At 4.5 m, 2.5 degrees is about 0.2 m sideways; the hub opening is about 1 m wide.
     */
    private static final double AIM_TOLERANCE_DEGREES = 2.5;

    // ---- Shoot on the move settings ----

    /**
     * Time from "decide to shoot" to "ball leaves the shooter": code loop + feeding the ball
     * through. The robot keeps moving during this, so we aim from where it WILL be.
     * Measure it from video if you can; 0.05 to 0.15 s is typical.
     */
    private static final double RELEASE_DELAY_SECONDS = 0.10;

    /** How many times to go around the aim point / time of flight loop. 3 is plenty. */
    private static final int AIM_ITERATIONS = 3;

    /**
     * Don't shoot when the robot drives faster than this (m/s). The faster we go, the more any
     * error in the time of flight table moves the ball. Raise it once shots on the move are good.
     */
    private static final double MAX_SHOOT_SPEED_MPS = 2.5;

    // ---- Ready-to-shoot tolerances ----
    private static final double FLYWHEEL_TOLERANCE_RPS = 2.0;
    private static final double HOOD_TOLERANCE_DEGREES = 1.5;
    private static final double MIN_FLYWHEEL_RPS = 1.0;

    // ---- Shot tables: distance to aim point (m) -> flywheel speed, hood angle, time of flight ----
    //
    // ALL VALUES ARE STARTING GUESSES. How to tune:
    //  1. Stand still at a distance (tape on the floor, measure from the robot's center to the hub's center).
    //  2. Change flywheel speed and hood angle until balls go in reliably. Write both down.
    //  3. Film a few shots at 60 fps from the side. Count frames from the ball leaving the shooter to
    //     it reaching the hub, divide by 60: that is the time of flight for that distance.
    //  4. Repeat every ~0.5-1 m across the range you shoot from, and put the numbers in here.
    // Between rows the value is interpolated in a straight line; outside the table the end value is used.
    // Flywheel units are the same as Shooter.targetShooterVelocity (rotations per second of the flywheel).

    private static final double MIN_TABLE_DISTANCE = 1.0;
    private static final double MAX_TABLE_DISTANCE = 6.0;

    private static final InterpolatingDoubleTreeMap HUB_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HUB_HOOD_DEGREES = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap HUB_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap PASS_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap PASS_HOOD_DEGREES = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap PASS_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    static {
        //            distance (m)  value
        HUB_FLYWHEEL_RPS.put(1.0, 38.0);
        HUB_FLYWHEEL_RPS.put(2.0, 44.0);
        HUB_FLYWHEEL_RPS.put(3.0, 50.0);
        HUB_FLYWHEEL_RPS.put(4.0, 57.0);
        HUB_FLYWHEEL_RPS.put(5.0, 64.0);
        HUB_FLYWHEEL_RPS.put(6.0, 71.0);

        HUB_HOOD_DEGREES.put(1.0, 8.0);
        HUB_HOOD_DEGREES.put(2.0, 15.0);
        HUB_HOOD_DEGREES.put(3.0, 22.0);
        HUB_HOOD_DEGREES.put(4.0, 28.0);
        HUB_HOOD_DEGREES.put(5.0, 33.0);
        HUB_HOOD_DEGREES.put(6.0, 37.0);

        HUB_TIME_OF_FLIGHT.put(1.0, 0.70);
        HUB_TIME_OF_FLIGHT.put(2.0, 0.80);
        HUB_TIME_OF_FLIGHT.put(3.0, 0.90);
        HUB_TIME_OF_FLIGHT.put(4.0, 1.00);
        HUB_TIME_OF_FLIGHT.put(5.0, 1.10);
        HUB_TIME_OF_FLIGHT.put(6.0, 1.20);

        // Passing: a lower, flatter shot that lands on the floor near our depot / outpost.
        PASS_FLYWHEEL_RPS.put(3.0, 40.0);
        PASS_FLYWHEEL_RPS.put(6.0, 55.0);
        PASS_FLYWHEEL_RPS.put(9.0, 70.0);
        PASS_HOOD_DEGREES.put(3.0, 30.0);
        PASS_HOOD_DEGREES.put(6.0, 38.0);
        PASS_HOOD_DEGREES.put(9.0, 45.0);
        PASS_TIME_OF_FLIGHT.put(3.0, 0.9);
        PASS_TIME_OF_FLIGHT.put(6.0, 1.2);
        PASS_TIME_OF_FLIGHT.put(9.0, 1.5);
    }

    // ---- Latest results (for the dashboard / isReadyToShoot) ----
    private double distance = 0;
    private double timeOfFlight = 0;
    private double robotSpeed = 0;

    public Shooter(Swerve swerve) {
        poseSlave = swerve;
        hoodMotor = TalonFXFactory.createDefaultTalon(Ports.HOOD_MOTOR);
        shooterMotor1 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR1);
        shooterMotor2 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR2);
        shooterMotor3 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR3);
        shooterMotor4 = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_MOTOR4);

        TalonUtil.applyAndCheckConfiguration(shooterMotor1, ShooterConstants.getFrontShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor2, ShooterConstants.getFrontShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor3, ShooterConstants.getBackShooterConfig());
        TalonUtil.applyAndCheckConfiguration(shooterMotor4, ShooterConstants.getBackShooterConfig());
        TalonUtil.applyAndCheckConfiguration(hoodMotor, ShooterConstants.getHoodConfig());
    }

    @Override
    public void periodic() {
        calculateTargetMain();

        if (autoAim) {
            setTargetVelocity(calculateVelocity());
            setTargetAngle(calculateHoodAngle());
        }

        shooterMotor1.setControl(motionMagic.withVelocity(targetShooterVelocity));
        shooterMotor2.setControl(motionMagic.withVelocity(targetShooterVelocity));
        shooterMotor3.setControl(motionMagic.withVelocity(targetShooterVelocity));
        shooterMotor4.setControl(motionMagic.withVelocity(targetShooterVelocity));

        hoodMotor.setControl(hoodMotionMagic.withPosition(targetHoodAngle));
        SmartDashboard.putNumber("shooter/TargetAngle", targetHoodAngle.in(Degrees));
        SmartDashboard.putNumber("shooter/Error", targetHoodAngle.in(Degrees) - getAngle().in(Degrees));
        SmartDashboard.putNumber("shooter/TargetShooterVelocity", targetShooterVelocity);
        SmartDashboard.putNumber("shooter/ShooterError", targetShooterVelocity - getVelocity());
        SmartDashboard.putNumber("shooter/AimDistance", distance);
        SmartDashboard.putNumber("shooter/TimeOfFlight", timeOfFlight);
        SmartDashboard.putNumber("shooter/RobotSpeed", robotSpeed);
        SmartDashboard.putNumber("shooter/AimPointX", aimPoint.getX());
        SmartDashboard.putNumber("shooter/AimPointY", aimPoint.getY());
        SmartDashboard.putNumber("shooter/TargetHeadingDegrees", targetHeading.getDegrees());
        SmartDashboard.putNumber("shooter/AimErrorDegrees", Math.toDegrees(aimErrorRadians));
        SmartDashboard.putBoolean("shooter/Aimed", isAimed());
        SmartDashboard.putBoolean("shooter/ReadyToShoot", isReadyToShoot());
    }

    public Angle getAngle() {
        return hoodMotor.getPosition().getValue();
    }

    /** Hood target. Only sticks when {@link #autoAim} is false (otherwise periodic() overwrites it). */
    public void setTargetAngle(Angle angle) {
        targetHoodAngle = angle;
    }

    public double getVelocity() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    /** Flywheel target. Only sticks when {@link #autoAim} is false (otherwise periodic() overwrites it). */
    public void setTargetVelocity(double velocity) {
        targetShooterVelocity = velocity;
    }

    /**
     * Field heading the robot should have so the shooter (out the back) points at the aim point.
     * Give this to the drivetrain's heading controller.
     */
    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    /** How far the robot is from {@link #getTargetHeading()}, -pi..pi radians. Positive = turn CCW. */
    public double getAimErrorRadians() {
        return aimErrorRadians;
    }

    /** True when the shooter points at the aim point within {@link #AIM_TOLERANCE_DEGREES}. */
    public boolean isAimed() {
        return Math.abs(aimErrorRadians) <= Math.toRadians(AIM_TOLERANCE_DEGREES);
    }

    /** Distance from the shooter to the aim point, in meters. */
    public double getAimDistance() {
        return distance;
    }

    /**
     * True when a ball fed now should go in: flywheel at speed, hood on target, robot pointed at the
     * aim point ({@link #isAimed()}), robot slower than {@link #MAX_SHOOT_SPEED_MPS}, and the aim
     * point inside the range the shot tables cover. (It does not check the alliance zone rule.)
     */
    public boolean isReadyToShoot() {
        boolean flywheelReady = targetShooterVelocity >= MIN_FLYWHEEL_RPS
                && Math.abs(getVelocity() - targetShooterVelocity) <= FLYWHEEL_TOLERANCE_RPS;
        boolean hoodReady = Math.abs(getAngle().in(Degrees) - targetHoodAngle.in(Degrees)) <= HOOD_TOLERANCE_DEGREES;
        boolean slowEnough = !shootOnTheMove ? robotSpeed < 0.1 : robotSpeed <= MAX_SHOOT_SPEED_MPS;
        boolean inRange = !targetHub || (distance >= MIN_TABLE_DISTANCE && distance <= MAX_TABLE_DISTANCE);
        return flywheelReady && hoodReady && isAimed() && slowEnough && inRange;
    }

    private void calculateTargetMain() {
        if (targetHub) calculateTargetHub();
        else calculateTargetPass();
    }

    private Angle calculateHoodAngle() {
        double degrees = (targetHub ? HUB_HOOD_DEGREES : PASS_HOOD_DEGREES).get(distance);
        double min = Math.min(ShooterConstants.lowerLimit.in(Degrees), ShooterConstants.upperLimit.in(Degrees));
        double max = Math.max(ShooterConstants.lowerLimit.in(Degrees), ShooterConstants.upperLimit.in(Degrees));
        return Degrees.of(MathUtil.clamp(degrees, min, max));
    }

    private double calculateVelocity() {
        return (targetHub ? HUB_FLYWHEEL_RPS : PASS_FLYWHEEL_RPS).get(distance);
    }

    private void calculateTargetHub() {
        aimAt(ourHub(), HUB_TIME_OF_FLIGHT);
    }

    private void calculateTargetPass() {
        Translation2d robot = poseSlave.getPose().getTranslation();
        Translation2d depot = forOurAlliance(new Translation2d(
                FieldConstants.Depot.depotCenter.getX(), FieldConstants.Depot.depotCenter.getY()));
        Translation2d outpost = forOurAlliance(FieldConstants.Outpost.centerPoint);
        // (The old code compared squared distances and then used the SQUARED distance as the distance.)
        Translation2d target = robot.getDistance(depot) < robot.getDistance(outpost) ? depot : outpost;
        aimAt(target, PASS_TIME_OF_FLIGHT);
    }

    /**
     * The shoot-on-the-move math. Finds the aim point for {@code target} and stores it in
     * {@link #aimPoint}, {@link #distance}, {@link #timeOfFlight}, {@link #targetHeading} and
     * {@link #aimErrorRadians}.
     */
    private void aimAt(Translation2d target, InterpolatingDoubleTreeMap timeOfFlightTable) {
        Pose2d pose = poseSlave.getPose();

        // getChassisSpeeds() is robot-relative (forward/left); turn it into field directions.
        ChassisSpeeds robotSpeeds = poseSlave.getChassisSpeeds();
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, pose.getRotation());
        robotSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        // The shooter's own position and velocity on the field. If it is off-center, turning the
        // robot swings it around: extra velocity = omega x offset.
        Translation2d offset = SHOOTER_OFFSET_METERS.rotateBy(pose.getRotation());
        double omega = robotSpeeds.omegaRadiansPerSecond;
        double vx = shootOnTheMove ? fieldSpeeds.vxMetersPerSecond - omega * offset.getY() : 0.0;
        double vy = shootOnTheMove ? fieldSpeeds.vyMetersPerSecond + omega * offset.getX() : 0.0;

        // Where the shooter will be when the ball actually leaves it.
        Translation2d shotFrom = pose.getTranslation().plus(offset).plus(
                new Translation2d(vx * RELEASE_DELAY_SECONDS, vy * RELEASE_DELAY_SECONDS));

        // Aim point = target shifted back by (robot velocity x time of flight). The time of flight
        // depends on the distance to the aim point, so repeat a few times until it settles.
        Translation2d aim = target;
        for (int i = 0; i < AIM_ITERATIONS; i++) {
            double tof = timeOfFlightTable.get(shotFrom.getDistance(aim));
            aim = target.minus(new Translation2d(vx * tof, vy * tof));
        }

        aimPoint = aim;
        distance = shotFrom.getDistance(aim);
        timeOfFlight = timeOfFlightTable.get(distance);
        // No turret: point the robot so the shooter side faces the aim point.
        targetHeading = aim.minus(shotFrom).getAngle().minus(Rotation2d.fromDegrees(SHOOTER_FACING_DEGREES));
        aimErrorRadians = MathUtil.angleModulus(targetHeading.getRadians() - pose.getRotation().getRadians());
    }

    /** Our alliance's hub. FieldConstants.Hub.topCenterPoint is the BLUE hub. */
    private Translation2d ourHub() {
        return forOurAlliance(new Translation2d(
                FieldConstants.Hub.topCenterPoint.getX(), FieldConstants.Hub.topCenterPoint.getY()));
    }

    /** Takes a blue-side field position and returns the same spot on our alliance's side. */
    private Translation2d forOurAlliance(Translation2d bluePosition) {
        if (poseSlave.getAlliance() == Alliance.Red) {
            return new Translation2d(
                    FieldConstants.fieldLength - bluePosition.getX(),
                    FieldConstants.fieldWidth - bluePosition.getY());
        }
        return bluePosition;
    }
}
