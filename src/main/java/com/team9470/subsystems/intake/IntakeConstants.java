package com.team9470.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

public class IntakeConstants {
        
    public static final TalonFXConfiguration kIntakeDeployConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRollerConfig = new TalonFXConfiguration();

    public static final double ROLLER_SPEED = 12.0;

    static {
        kRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kRollerConfig.CurrentLimits;

        kIntakeDeployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
}
