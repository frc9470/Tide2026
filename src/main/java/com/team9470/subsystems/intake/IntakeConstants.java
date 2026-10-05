package com.team9470.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
        
    public static final TalonFXConfiguration kRightIntakeDeployConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kLeftIntakeDeployConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRightRollerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kLeftRollerConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kKickerRollerConfig = new TalonFXConfiguration();

    public static final double ROLLER_SPEED = 12.0;

    static {

        //Left Roller
        kLeftRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kLeftRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // kLeftRollerConfig.CurrentLimits;dont worry about this


        //Right Roller
        kRightRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kRightRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //Right Intake Deploy
        kRightIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kRightIntakeDeployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //Left Intake Deploy
        kLeftIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kLeftIntakeDeployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //Kicker Roller
        kKickerRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kKickerRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
}
