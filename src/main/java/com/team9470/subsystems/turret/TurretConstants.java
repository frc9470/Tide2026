package com.team9470.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class TurretConstants {
    private TurretConstants() {
    }
    public static final Angle upperLimit = Units.Rotations.of(0);
    public static final Angle lowerLimit = Units.Rotations.of(1);
    public static final double kP = 0;
    public static final double kD = 0;
    public static final double CRUISE_VELOCITY = 2;
    public static final double ACCELERATION = 15; 
    public static final double JERK = 0; 

    public static final double GEAR_RATIO = 8;

    public static TalonFXConfiguration getTurretConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
        config.MotionMagic.MotionMagicJerk = JERK;
        config.Slot0.kP = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        return config;
    }
}
