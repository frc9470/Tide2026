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
}
