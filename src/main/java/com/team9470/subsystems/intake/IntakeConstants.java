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
    public static final double kHomingVoltage = 5.0;
    public static final double kStallCurrent = 30.0;
    public static final double kStallVelocity = 0.1;
    public static final double kStallTime = 0.2;
    public static final double kHome = 1.0;

    public static final double ROLLER_SPEED = 10;

    public static final double DEPLOYED_POSITION_ROTATIONS = 1.0;
    public static final double AGITATE_POSITION_ROTATIONS = .6;

    public static final double KICKER_SPEED = 6;

    public static final int kS = 0;
    public static final int kV = 0;
    public static final int kP = 30;

    public static final double INTAKE_RACK_GEAR_RATIO = (36.0 / 10.0) * (50.0 / 10.0);


    public static TalonFXConfiguration getReverseConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        return config;
    }
    static {

        //Right Roller
        kRightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kRightRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kRightRollerConfig.CurrentLimits.StatorCurrentLimit = kRollerStatorCurrentLimit;


        //Left Roller
        kLeftIntakeDeployConfig.Slot0.kS = kS;
        kLeftIntakeDeployConfig.Slot0.kV = kV;
        kLeftIntakeDeployConfig.Slot0.kP = kP;
        kLeftIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kLeftIntakeDeployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kLeftIntakeDeployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kLeftIntakeDeployConfig.CurrentLimits.StatorCurrentLimit = kIntakeStatorCurrentLimit;
        kLeftIntakeDeployConfig.Feedback.SensorToMechanismRatio = INTAKE_RACK_GEAR_RATIO;
        // kLeftRollerConfig.CurrentLimits;dont worry about this

        //Right Intake Deploy
        kRightIntakeDeployConfig.Slot0.kS = kS;
        kRightIntakeDeployConfig.Slot0.kV = kV;
        kRightIntakeDeployConfig.Slot0.kP = kP;
        kRightIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kRightIntakeDeployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kRightIntakeDeployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kRightIntakeDeployConfig.CurrentLimits.StatorCurrentLimit = kIntakeStatorCurrentLimit;
        kRightIntakeDeployConfig.Feedback.SensorToMechanismRatio = INTAKE_RACK_GEAR_RATIO;

        //Left Intake Deploy
        kLeftIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kLeftIntakeDeployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //Kicker Roller
        kKickerRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kKickerRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}
