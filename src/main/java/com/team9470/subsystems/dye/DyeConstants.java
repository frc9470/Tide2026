package com.team9470.subsystems.dye;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class DyeConstants {
    private DyeConstants() {
    }

    public static final double FEED_VELOCITY_RPS = 0;
    public static final double REVERSE_VELOCITY_RPS = 0;
    public static final double VELOCITY_TOLERANCE_RPS = 1;

    public static final double GEAR_RATIO = 1;
    public static final double STATOR_CURRENT_LIMIT = 40;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 0;

    // Rollers: left is the leader, right follows. Both gears drive the same belt.
    // Set true if the right motor must spin opposite the left (mirrored mounting).
    public static final boolean RIGHT_ROLLER_OPPOSES_LEFT = true;
    public static final double ROLLER_FEED_VELOCITY_RPS = 0;
    public static final double ROLLER_REVERSE_VELOCITY_RPS = 0;
    public static final double ROLLER_VELOCITY_TOLERANCE_RPS = 1;
    public static final double ROLLER_GEAR_RATIO = 1;
    public static final double ROLLER_STATOR_CURRENT_LIMIT = 40;
    public static final double ROLLER_kS = 0;
    public static final double ROLLER_kV = 0;
    public static final double ROLLER_kP = 0;

    public static TalonFXConfiguration getConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kP = kP;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        return config;
    }

    /** Config for the leader roller; the follower copies its output. */
    public static TalonFXConfiguration getRollerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = ROLLER_kS;
        config.Slot0.kV = ROLLER_kV;
        config.Slot0.kP = ROLLER_kP;
        config.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_CURRENT_LIMIT;
        return config;
    }

    /** Config for the follower roller (current limit/brake only; direction comes from Follower alignment). */
    public static TalonFXConfiguration getRollerFollowerConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ROLLER_STATOR_CURRENT_LIMIT;
        return config;
    }
}
