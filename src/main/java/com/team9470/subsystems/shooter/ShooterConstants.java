package com.team9470.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class ShooterConstants {
    private ShooterConstants() {
    }
    public static final Angle upperLimit = Units.Degrees.of(0);
    public static final Angle lowerLimit = Units.Degrees.of(50);
    public static final double CRUISE_VELOCITY = 2;
    public static final double ACCELERATION = 15; 
    public static final double JERK = 0; 

    public static final double kP = 100.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    
    public static final double hoodkP = 40.0;
    public static final double hoodkD = 0.0;
    public static final double hoodkG = 0.0;
    public static final double hoodkS = 0.37;

    private static final double kHoodStatorCurrentLimit = 40.0;
    private static final double kShooterStatorCurrentLimit = 40.0;

    public static final double kShooterVelocityTolerance = 2.0;
    public static final double kHoodAngleTolerance = 5.0;

    //hood homing
    public static double kHome = 0.0;
    public static double kHomingVoltage = -3.0;
    public static double kStallCurrent = 20.0;
    public static double kStallVelocity = 2.0;
    public static double kStallTime = 0.05;
    public static double tof;


    public static final double SHOOTER_GEAR_RATIO1 = 20.0 / 36.0; 
    public static final double SHOOTER_GEAR_RATIO2 = 1;
    public static final double HOOD_GEAR_RATIO = (290.0 / 22.0) * (22.0 / 20.0) * (24.0 / 8.0); 

    public static final TalonFXConfiguration kHoodConfig = new TalonFXConfiguration();

    public static TalonFXConfiguration getFrontShooterConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
        config.MotionMagic.MotionMagicJerk = JERK;
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Feedback.SensorToMechanismRatio = SHOOTER_GEAR_RATIO1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        return config;
    }

    public static TalonFXConfiguration getBackShooterConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
        config.MotionMagic.MotionMagicJerk = JERK;
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Feedback.SensorToMechanismRatio = SHOOTER_GEAR_RATIO2;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = kShooterStatorCurrentLimit;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        return config;
    }
    static{
        kHoodConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        kHoodConfig.MotionMagic.MotionMagicAcceleration = ACCELERATION;
        kHoodConfig.MotionMagic.MotionMagicJerk = JERK;
        kHoodConfig.Slot0.kP = hoodkP;
        kHoodConfig.Slot0.kD = hoodkD;
        kHoodConfig.Slot0.kS = hoodkS;
        kHoodConfig.Slot0.kG = hoodkG;
        kHoodConfig.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        kHoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kHoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kHoodConfig.CurrentLimits.StatorCurrentLimit = kHoodStatorCurrentLimit;
        kHoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}
