package com.team9470.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Ports;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private final TalonFX rollerMotor;
    private final TalonFX intakeMotor;

    public boolean deployed = false;

    
    public Intake() {
        rollerMotor = TalonFXFactory.createDefaultTalon(Ports.ROLLER_MOTOR);
        intakeMotor = TalonFXFactory.createDefaultTalon(Ports.INTAKE_MOTOR);

        TalonUtil.applyAndCheckConfiguration(rollerMotor, IntakeConstants.kRollerConfig);
        TalonUtil.applyAndCheckConfiguration(intakeMotor, IntakeConstants.kIntakeDeployConfig);
    }
    
    @Override
    public void periodic() {


        double rollerVolts;
        if(deployed){
            rollerVolts = IntakeConstants.ROLLER_SPEED;
        }
        else {
            rollerVolts = 0.0;
        }
        rollerMotor.setVoltage(rollerVolts);
    }

    public void stow() {
        deployed = false;
    }

    public void deploy() {

    }


}
