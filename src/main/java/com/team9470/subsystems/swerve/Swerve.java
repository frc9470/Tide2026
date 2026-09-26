package com.team9470.subsystems.swerve;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.team9470.Constants;
import com.team9470.FieldConstants;
import com.team9470.TunerConstants;
import com.team9470.Telemetry;
import com.team9470.TunerConstants.TunerSwerveDrivetrain;
import com.team9470.telemetry.TelemetryManager;
import com.team9470.subsystems.vision.VisionPoseAcceptor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Arrays;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {

    private final VisionPoseAcceptor visionPoseAcceptor = new VisionPoseAcceptor();
    private static Swerve instance;
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    private final TelemetryManager telemetry = TelemetryManager.getInstance();

    // Cached SmartDashboard gain values (drive Slot0)
    private double cachedDriveKP, cachedDriveKI, cachedDriveKD, cachedDriveKS, cachedDriveKV;
    // Cached SmartDashboard gain values (steer Slot0)
    private double cachedSteerKP, cachedSteerKI, cachedSteerKD, cachedSteerKS, cachedSteerKV, cachedSteerKA;
    // Cached SmartDashboard gain values (Choreo PID)
    private double cachedChoreoXYkP, cachedChoreoXYkI, cachedChoreoXYkD;
    private double cachedChoreoThetakP, cachedChoreoThetakI, cachedChoreoThetakD;

    private static final double AUTO_PATH_SAMPLE_TIMEOUT_SEC = 0.25;
    private static final double AUTO_STARTUP_MOTION_THRESHOLD_MPS = 0.05;
    private static final double AUTO_STARTUP_OMEGA_THRESHOLD_RAD_PER_SEC = 0.10;
    private static final double AUTO_CENTERLINE_TOUCH_DISTANCE_M = Constants.RobotGeometry.kChoreoSideBumperMeters;
    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
    private static final double[] NO_WHEEL_FORCE_FEEDFORWARDS = new double[0];
    private static final SwerveModule.DriveRequestType CHASSIS_SPEED_DRIVE_REQUEST_TYPE = Utils.isSimulation()
            ? SwerveModule.DriveRequestType.OpenLoopVoltage
            : SwerveModule.DriveRequestType.Velocity;
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.k180deg;

    /* Swerve requests */
    private final SwerveRequest.ApplyFieldSpeeds applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
            .withDriveRequestType(CHASSIS_SPEED_DRIVE_REQUEST_TYPE);
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(CHASSIS_SPEED_DRIVE_REQUEST_TYPE);
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private double activeDriveStatorCurrentLimitAmps = Double.NaN;
    private boolean turboDriveCurrentLimitEnabled = false;
    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(translationCharacterization.withVolts(output)),
                    null,
                    this));
    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(steerCharacterization.withVolts(volts)),
                    null,
                    this));
    /* The SysId routine to test */
    private final SysIdRoutine sysIdRoutineToApply = sysIdRoutineTranslation;
    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    private double m_lastSimTime;
    private double lastAutoPathSampleTimestampSec = Double.NEGATIVE_INFINITY;
    private Pose2d lastAutoDesiredPose = Pose2d.kZero;
    private double lastAutoDesiredRobotTimestampSec = Double.NaN;
    private boolean autoDesiredPoseSeen = false;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // private RobotConfig config;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        instance = this;
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // try {
        // config = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        // e.printStackTrace();
        // }
        // configAutoBuilder();
        initSmartDashboardGains();
        setDriveStatorCurrentLimitAmps(TunerConstants.kNominalSlipCurrent.in(Amps));
    }

    private void initSmartDashboardGains() {
        // Drive gains from TunerConstants
        cachedDriveKP = TunerConstants.driveGains.kP;
        cachedDriveKI = TunerConstants.driveGains.kI;
        cachedDriveKD = TunerConstants.driveGains.kD;
        cachedDriveKS = TunerConstants.driveGains.kS;
        cachedDriveKV = TunerConstants.driveGains.kV;
        SmartDashboard.putNumber("Drive/kP", cachedDriveKP);
        SmartDashboard.putNumber("Drive/kI", cachedDriveKI);
        SmartDashboard.putNumber("Drive/kD", cachedDriveKD);
        SmartDashboard.putNumber("Drive/kS", cachedDriveKS);
        SmartDashboard.putNumber("Drive/kV", cachedDriveKV);

        // Steer gains from TunerConstants
        cachedSteerKP = TunerConstants.steerGains.kP;
        cachedSteerKI = TunerConstants.steerGains.kI;
        cachedSteerKD = TunerConstants.steerGains.kD;
        cachedSteerKS = TunerConstants.steerGains.kS;
        cachedSteerKV = TunerConstants.steerGains.kV;
        cachedSteerKA = TunerConstants.steerGains.kA;
        SmartDashboard.putNumber("Steer/kP", cachedSteerKP);
        SmartDashboard.putNumber("Steer/kI", cachedSteerKI);
        SmartDashboard.putNumber("Steer/kD", cachedSteerKD);
        SmartDashboard.putNumber("Steer/kS", cachedSteerKS);
        SmartDashboard.putNumber("Steer/kV", cachedSteerKV);
        SmartDashboard.putNumber("Steer/kA", cachedSteerKA);

        // Choreo path-following PID
        cachedChoreoXYkP = pathXController.getP();
        cachedChoreoXYkI = pathXController.getI();
        cachedChoreoXYkD = pathXController.getD();
        cachedChoreoThetakP = pathThetaController.getP();
        cachedChoreoThetakI = pathThetaController.getI();
        cachedChoreoThetakD = pathThetaController.getD();
        SmartDashboard.putNumber("Choreo/XY_kP", cachedChoreoXYkP);
        SmartDashboard.putNumber("Choreo/XY_kI", cachedChoreoXYkI);
        SmartDashboard.putNumber("Choreo/XY_kD", cachedChoreoXYkD);
        SmartDashboard.putNumber("Choreo/Theta_kP", cachedChoreoThetakP);
        SmartDashboard.putNumber("Choreo/Theta_kI", cachedChoreoThetakI);
        SmartDashboard.putNumber("Choreo/Theta_kD", cachedChoreoThetakD);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {
        });
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(Choreo.TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                () -> getState().Pose,
                this::resetPose,
                this::followPath,
                true,
                this,
                trajLogger);
    }

    // For pathplanner
    // public void configAutoBuilder() {
    // AutoBuilder.configure(
    // this::getPose, // Robot pose supplier
    // this::resetPose, // Method to reset odometry (will be called if your auto has
    // a starting pose)
    // this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    // (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive
    // the robot given ROBOT
    // // RELATIVE ChassisSpeeds. Also optionally outputs
    // // individual module feedforwards
    // new PPHolonomicDriveController( // PPHolonomicController is the built in path
    // following controller for
    // // holonomic drive trains
    // new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
    // new PIDConstants(7.0, 0.0, 0.0) // Rotation PID constants
    // ),
    // config, // The robot configuration
    // () -> {
    // // Boolean supplier that controls when the path will be mirrored for the red
    // // alliance
    // // This will flip the path being followed to the red side of the field.
    // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    // return getAlliance() == Alliance.Red;
    // },
    // this // Reference to this subsystem to set requirements
    // );
    // }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setControl(
                robotCentricRequest.withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    /**
     * Drive with field-relative speeds, accounting for operator perspective.
     * Use this instead of setChassisSpeeds when the caller is providing
     * field-relative velocities (e.g. shoot-on-the-move translation).
     */
    public void setFieldSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        setControl(
                fieldCentricRequest.withVelocityX(vxMetersPerSecond)
                        .withVelocityY(vyMetersPerSecond)
                        .withRotationalRate(omegaRadiansPerSecond));
    }

    /**
     * X-lock all swerve modules (defensive stance).
     * Used during SOTM O-Lock when near-stationary and aligned.
     */
    public void stopWithXLock() {
        setControl(brakeRequest);
    }

    /**
     * Drive with actual field-relative ChassisSpeeds (blue-origin coordinate system).
     * Unlike setFieldSpeeds, this does NOT apply operator perspective.
     * Used for COR-shifted SOTM driving.
     */
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        setControl(applyFieldSpeeds.withSpeeds(fieldRelativeSpeeds)
                .withWheelForceFeedforwardsX(NO_WHEEL_FORCE_FEEDFORWARDS)
                .withWheelForceFeedforwardsY(NO_WHEEL_FORCE_FEEDFORWARDS));
    }

    public void setTurboDriveCurrentLimitEnabled(boolean enabled) {
        turboDriveCurrentLimitEnabled = enabled;
        setDriveStatorCurrentLimitAmps(
                enabled
                        ? TunerConstants.kTurboSlipCurrent.in(Amps)
                        : TunerConstants.kNominalSlipCurrent.in(Amps));
    }

    private void setDriveStatorCurrentLimitAmps(double statorCurrentLimitAmps) {
        boolean currentLimitChanged = Math.abs(statorCurrentLimitAmps - activeDriveStatorCurrentLimitAmps) >= 1e-9;
        activeDriveStatorCurrentLimitAmps = statorCurrentLimitAmps;
        if (currentLimitChanged) {
            CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(statorCurrentLimitAmps)
                    .withStatorCurrentLimitEnable(true);
            for (var module : getModules()) {
                module.getDriveMotor().getConfigurator().apply(driveCurrentLimits);
            }
        }
        telemetry.publishDriveCurrentLimits(
                activeDriveStatorCurrentLimitAmps,
                TunerConstants.kNominalSlipCurrent.in(Amps),
                TunerConstants.kTurboSlipCurrent.in(Amps),
                turboDriveCurrentLimitEnabled);
    }

    public DriverStation.Alliance getAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.orElse(Alliance.Blue);
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var state = getState();
        Pose2d measuredPose = state.Pose;
        Pose2d desiredPose = new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading));
        ChassisSpeeds feedforwardSpeeds = sample.getChassisSpeeds();

        double xFeedback = pathXController.calculate(measuredPose.getX(), sample.x);
        double yFeedback = pathYController.calculate(measuredPose.getY(), sample.y);
        double thetaFeedback = pathThetaController.calculate(
                measuredPose.getRotation().getRadians(), sample.heading);

        ChassisSpeeds feedbackSpeeds = new ChassisSpeeds(xFeedback, yFeedback, thetaFeedback);
        ChassisSpeeds commandedSpeeds = new ChassisSpeeds(
                feedforwardSpeeds.vxMetersPerSecond + xFeedback,
                feedforwardSpeeds.vyMetersPerSecond + yFeedback,
                feedforwardSpeeds.omegaRadiansPerSecond + thetaFeedback);
        Translation2d poseError = desiredPose.getTranslation().minus(measuredPose.getTranslation());
        double headingErrorRad = MathUtil.angleModulus(
                desiredPose.getRotation().getRadians() - measuredPose.getRotation().getRadians());
        double[] moduleForcesX = Utils.isSimulation() ? NO_WHEEL_FORCE_FEEDFORWARDS : sample.moduleForcesX();
        double[] moduleForcesY = Utils.isSimulation() ? NO_WHEEL_FORCE_FEEDFORWARDS : sample.moduleForcesY();
        lastAutoPathSampleTimestampSec = Timer.getTimestamp();
        markAutoCenterlineTouch(lastAutoPathSampleTimestampSec, desiredPose);
        telemetry.markDriveAutoFirstPathSample(lastAutoPathSampleTimestampSec);
        double commandedTranslationMps = Math.hypot(
                commandedSpeeds.vxMetersPerSecond,
                commandedSpeeds.vyMetersPerSecond);
        if (commandedTranslationMps >= AUTO_STARTUP_MOTION_THRESHOLD_MPS
                || Math.abs(commandedSpeeds.omegaRadiansPerSecond) >= AUTO_STARTUP_OMEGA_THRESHOLD_RAD_PER_SEC) {
            telemetry.markDriveAutoFirstMotionCommand(lastAutoPathSampleTimestampSec, commandedSpeeds);
        }

        telemetry.publishDriveAutoPathSample(
                lastAutoPathSampleTimestampSec,
                desiredPose,
                measuredPose,
                poseError,
                headingErrorRad,
                feedforwardSpeeds,
                feedbackSpeeds,
                commandedSpeeds,
                state.Speeds,
                moduleForcesX,
                moduleForcesY);

        setControl(
                applyFieldSpeeds.withSpeeds(commandedSpeeds)
                        .withWheelForceFeedforwardsX(moduleForcesX)
                        .withWheelForceFeedforwardsY(moduleForcesY));
    }

    public void resetAutoPathTracking() {
        lastAutoPathSampleTimestampSec = Double.NEGATIVE_INFINITY;
        lastAutoDesiredPose = Pose2d.kZero;
        lastAutoDesiredRobotTimestampSec = Double.NaN;
        autoDesiredPoseSeen = false;
    }

    private void markAutoCenterlineTouch(double currentRobotTimestampSec, Pose2d desiredPose) {
        double centerlineX = FieldConstants.LinesVertical.center;
        if (!autoDesiredPoseSeen) {
            autoDesiredPoseSeen = true;
            lastAutoDesiredPose = desiredPose;
            lastAutoDesiredRobotTimestampSec = currentRobotTimestampSec;
            if (desiredPose.getX() >= centerlineX - AUTO_CENTERLINE_TOUCH_DISTANCE_M) {
                telemetry.markDriveAutoCenterlineTouch(currentRobotTimestampSec, desiredPose);
            }
            return;
        }

        double touchThresholdX = centerlineX - AUTO_CENTERLINE_TOUCH_DISTANCE_M;
        double previousTouchX = lastAutoDesiredPose.getX();
        double currentTouchX = desiredPose.getX();
        if (previousTouchX < touchThresholdX && currentTouchX >= touchThresholdX) {
            double fraction = interpolationFraction(previousTouchX, currentTouchX, touchThresholdX);
            telemetry.markDriveAutoCenterlineTouch(
                    interpolate(lastAutoDesiredRobotTimestampSec, currentRobotTimestampSec, fraction),
                    lastAutoDesiredPose.interpolate(desiredPose, fraction));
        }

        lastAutoDesiredPose = desiredPose;
        lastAutoDesiredRobotTimestampSec = currentRobotTimestampSec;
    }

    private double interpolationFraction(double start, double end, double target) {
        double denominator = end - start;
        if (Math.abs(denominator) < 1e-9) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, (target - start) / denominator));
    }

    private double interpolate(double start, double end, double fraction) {
        return start + (end - start) * fraction;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        logger.telemeterize(getState());

        var modules = getModules();
        double[] driveVelocityRps = new double[modules.length];
        double[] driveStatorCurrentAmps = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            var driveMotor = modules[i].getDriveMotor();
            if (Utils.isSimulation()) {
                driveVelocityRps[i] = driveMotor.getRotorVelocity().getValueAsDouble();
                driveStatorCurrentAmps[i] = driveMotor.getStatorCurrent().getValueAsDouble();
            } else {
                driveVelocityRps[i] = driveMotor.getRotorVelocity().refresh().getValueAsDouble();
                driveStatorCurrentAmps[i] = driveMotor.getStatorCurrent().refresh().getValueAsDouble();
            }
        }
        telemetry.publishDriveModuleElectrical(driveVelocityRps, driveStatorCurrentAmps);

        boolean autoPathActive = DriverStation.isAutonomous() && DriverStation.isEnabled()
                && (Timer.getTimestamp() - lastAutoPathSampleTimestampSec) <= AUTO_PATH_SAMPLE_TIMEOUT_SEC;
        telemetry.publishDriveAutoPathActive(autoPathActive);
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? RED_ALLIANCE_PERSPECTIVE_ROTATION
                                : BLUE_ALLIANCE_PERSPECTIVE_ROTATION);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        updateSmartDashboardGains();
    }

    private void updateSmartDashboardGains() {
        // --- Drive gains ---
        double driveKP = SmartDashboard.getNumber("Drive/kP", cachedDriveKP);
        double driveKI = SmartDashboard.getNumber("Drive/kI", cachedDriveKI);
        double driveKD = SmartDashboard.getNumber("Drive/kD", cachedDriveKD);
        double driveKS = SmartDashboard.getNumber("Drive/kS", cachedDriveKS);
        double driveKV = SmartDashboard.getNumber("Drive/kV", cachedDriveKV);
        if (driveKP != cachedDriveKP || driveKI != cachedDriveKI || driveKD != cachedDriveKD
                || driveKS != cachedDriveKS || driveKV != cachedDriveKV) {
            cachedDriveKP = driveKP;
            cachedDriveKI = driveKI;
            cachedDriveKD = driveKD;
            cachedDriveKS = driveKS;
            cachedDriveKV = driveKV;
            Slot0Configs newDriveGains = new Slot0Configs()
                    .withKP(driveKP).withKI(driveKI).withKD(driveKD)
                    .withKS(driveKS).withKV(driveKV);
            for (var module : getModules()) {
                module.getDriveMotor().getConfigurator().apply(newDriveGains);
            }
        }

        // --- Steer gains ---
        double steerKP = SmartDashboard.getNumber("Steer/kP", cachedSteerKP);
        double steerKI = SmartDashboard.getNumber("Steer/kI", cachedSteerKI);
        double steerKD = SmartDashboard.getNumber("Steer/kD", cachedSteerKD);
        double steerKS = SmartDashboard.getNumber("Steer/kS", cachedSteerKS);
        double steerKV = SmartDashboard.getNumber("Steer/kV", cachedSteerKV);
        double steerKA = SmartDashboard.getNumber("Steer/kA", cachedSteerKA);
        if (steerKP != cachedSteerKP || steerKI != cachedSteerKI || steerKD != cachedSteerKD
                || steerKS != cachedSteerKS || steerKV != cachedSteerKV || steerKA != cachedSteerKA) {
            cachedSteerKP = steerKP;
            cachedSteerKI = steerKI;
            cachedSteerKD = steerKD;
            cachedSteerKS = steerKS;
            cachedSteerKV = steerKV;
            cachedSteerKA = steerKA;
            Slot0Configs newSteerGains = new Slot0Configs()
                    .withKP(steerKP).withKI(steerKI).withKD(steerKD)
                    .withKS(steerKS).withKV(steerKV).withKA(steerKA);
            for (var module : getModules()) {
                module.getSteerMotor().getConfigurator().apply(newSteerGains);
            }
        }

        // --- Choreo PID ---
        double choreoXYkP = SmartDashboard.getNumber("Choreo/XY_kP", cachedChoreoXYkP);
        double choreoXYkI = SmartDashboard.getNumber("Choreo/XY_kI", cachedChoreoXYkI);
        double choreoXYkD = SmartDashboard.getNumber("Choreo/XY_kD", cachedChoreoXYkD);
        double choreoThetakP = SmartDashboard.getNumber("Choreo/Theta_kP", cachedChoreoThetakP);
        double choreoThetakI = SmartDashboard.getNumber("Choreo/Theta_kI", cachedChoreoThetakI);
        double choreoThetakD = SmartDashboard.getNumber("Choreo/Theta_kD", cachedChoreoThetakD);
        if (choreoXYkP != cachedChoreoXYkP || choreoXYkI != cachedChoreoXYkI || choreoXYkD != cachedChoreoXYkD) {
            cachedChoreoXYkP = choreoXYkP;
            cachedChoreoXYkI = choreoXYkI;
            cachedChoreoXYkD = choreoXYkD;
            pathXController.setPID(choreoXYkP, choreoXYkI, choreoXYkD);
            pathYController.setPID(choreoXYkP, choreoXYkI, choreoXYkD);
        }
        if (choreoThetakP != cachedChoreoThetakP || choreoThetakI != cachedChoreoThetakI
                || choreoThetakD != cachedChoreoThetakD) {
            cachedChoreoThetakP = choreoThetakP;
            cachedChoreoThetakI = choreoThetakI;
            cachedChoreoThetakD = choreoThetakD;
            pathThetaController.setPID(choreoThetakP, choreoThetakI, choreoThetakD);
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        /* use the measured time delta, get battery voltage from WPILib */
        Notifier m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Rotation2d getGyroHeading() {
        return getState().RawHeading;
    }

    public ChassisSpeeds getChassisSpeeds() {
        // Get the current swerve module states
        SwerveModuleState[] moduleStates = getState().ModuleStates;

        // Convert module states to chassis speeds using the drivetrain kinematics
        SwerveDriveKinematics kinematics = getKinematics();
        return kinematics.toChassisSpeeds(moduleStates);
    }

    public double[] getWheelRadiusCharacterizationPosition() {
        return Arrays.stream(getState().ModulePositions)
                .mapToDouble(position -> position.distanceMeters)
                .toArray();
    }

    public static Swerve getInstance() {
        if (instance == null) {
            instance = TunerConstants.createDrivetrain();
        }
        return instance;
    }

    /**
     * Get Twist2d of robot velocity
     */
    // public RobotConfig getRobotConfig() {
    // return config;
    // }
    public Twist2d getRobotTwist() {
        return new Twist2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond,
                getChassisSpeeds().omegaRadiansPerSecond);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        if (visionPoseAcceptor.shouldAcceptVision(timestampSeconds, visionRobotPoseMeters, getPose(), getRobotTwist(),
                DriverStation.isAutonomous() && !DriverStation.isDisabled())) {
            super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        }
    }

}
