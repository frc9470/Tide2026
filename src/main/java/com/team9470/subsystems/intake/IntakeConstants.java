package com.team9470.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class IntakeConstants {
        
    public static final TalonFXConfiguration kRightIntakeDeployConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kLeftIntakeDeployConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRightRollerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kLeftRollerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kKickerRollerConfig = new TalonFXConfiguration();


    //fix
    public static final double kIntakeStatorCurrentLimit = 40.0;
    public static final double kIntakeSupplyCurrentLimit = 40.0;

    public static final double kRollerStatorCurrentLimit = 40.0;
    public static final double kRollerSupplyCurrentLimit = 40.0;

    public static final double kKickerStatorCurrentLimit = 40.0;
    public static final double kKickerSupplyCurrentLimit = 40.0;

    //homing constants
    public static final double kHomingVoltage = -3.0;
    public static final double kStallCurrent = 20.0;
    public static final double kStallVelocity = 0.1;
    public static final double kStallTime = 0.05;
    public static final double kHome = 0.0;

    public static final double ROLLER_SPEED = 10;

    public static final double DEPLOYED_POSITION_ROTATIONS = -12.0;
    public static final double AGITATE_POSITION_ROTATIONS = -10.0;

    public static final double KICKER_SPEED = 6;

    public static final int kS = 0;
    public static final int kV = 0;
    public static final int kP = 2;

    public static TalonFXConfiguration getConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kP = kP;
        config.Feedback.SensorToMechanismRatio = 1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = kIntakeStatorCurrentLimit;
        return config;
    }
    public static TalonFXConfiguration getReverseConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kP = kP;
        config.Feedback.SensorToMechanismRatio = 1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = kIntakeStatorCurrentLimit;
        return config;
    }
    static {

        //Right Roller
        kRightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kRightRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kRightRollerConfig.CurrentLimits.StatorCurrentLimit = kRollerStatorCurrentLimit;


        //Left Roller
        kLeftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kLeftRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // kLeftRollerConfig.CurrentLimits;dont worry about this

        //Right Intake Deploy
        kRightIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kRightIntakeDeployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //Left Intake Deploy
        kLeftIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kLeftIntakeDeployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //Kicker Roller
        kKickerRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kKickerRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}
