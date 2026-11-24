package com.team9470.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class TurretConstants {
    private TurretConstants() {
    }
    public static final Angle upperLimit = Units.Degrees.of(-225);
    public static final Angle lowerLimit = Units.Degrees.of(135);
    public static final double tof = 0.6;
    public static final double speedCap = 0.6;
    public static final double kP = 50;
    public static final double kD = 0.5;
    public static final double kS = 0.37;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kI = 10;
    public static final double kG = 0;

    public static final double kTolerance = 0.5;

    public static final TalonFXConfiguration kTurretConfig = new TalonFXConfiguration();

    public static final double TURRET_TO_GEAR_RATIO = (137.0 / 28.0) * (28.0 / 10.0) * (40.0 / 14.0);

    static{
        kTurretConfig.Slot0.kG = kG;
        kTurretConfig.Slot0.kI = kI;
        kTurretConfig.Slot0.kP = kP;
        kTurretConfig.Slot0.kD = kD;
        kTurretConfig.Slot0.kS = kS;
        kTurretConfig.Slot0.kV = kV;
        kTurretConfig.Slot0.kA = kD;
        kTurretConfig.Feedback.SensorToMechanismRatio = TURRET_TO_GEAR_RATIO;
        kTurretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kTurretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kTurretConfig.CurrentLimits.StatorCurrentLimit = 40;
        kTurretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kTurretConfig.Slot0.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    

    }
}
