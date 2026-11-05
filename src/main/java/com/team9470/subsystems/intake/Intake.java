package com.team9470.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Ports;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private final TalonFX rightRollerMotor;
    private final TalonFX leftRollerMotor;
    private final TalonFX rightIntakeMotor;
    private final TalonFX leftIntakeMotor;
    private final TalonFX kickerMotor;

    private final PositionVoltage intakePositionRequest = new PositionVoltage(0);

    public boolean deployed = false;
    public boolean agitating = false;
    public boolean rollersRunning = false;
    private Timer timer = new Timer();

    public Intake() {
        rightRollerMotor = TalonFXFactory.createDefaultTalon(Ports.RIGHT_ROLLER_MOTOR);
        leftRollerMotor = TalonFXFactory.createDefaultTalon(Ports.LEFT_ROLLER_MOTOR);

        rightIntakeMotor = TalonFXFactory.createDefaultTalon(Ports.RIGHT_DEPLOY_MOTOR);
        leftIntakeMotor = TalonFXFactory.createDefaultTalon(Ports.LEFT_DEPLOY_MOTOR);

        kickerMotor = TalonFXFactory.createDefaultTalon(Ports.KICKER_ROLLER);

        TalonUtil.applyAndCheckConfiguration(rightRollerMotor, IntakeConstants.kRightRollerConfig);
        TalonUtil.applyAndCheckConfiguration(leftRollerMotor, IntakeConstants.kLeftRollerConfig);

        TalonUtil.applyAndCheckConfiguration(rightIntakeMotor, IntakeConstants.getConfig());
        TalonUtil.applyAndCheckConfiguration(leftIntakeMotor, IntakeConstants.getReverseConfig());

        TalonUtil.applyAndCheckConfiguration(kickerMotor, IntakeConstants.kKickerRollerConfig);
    }

    @Override
    public void periodic() {


        double rollerVolts;
        double kickerVolts;
        if(deployed || agitating){
            rollerVolts = IntakeConstants.ROLLER_SPEED;
            kickerVolts = IntakeConstants.KICKER_SPEED;
        }
        else {
            rollerVolts = 0.0;
            kickerVolts = 0.0;
        }

        if(agitating && timer.get() % 1.0 - 0.5 <= 0){
            stow();
        }else if(agitating){
            deploy();
        }
        
        rightRollerMotor.setVoltage(rollerVolts);
        kickerMotor.setVoltage(kickerVolts);
    }

     public void deploy() {
        rightIntakeMotor.setControl(
            intakePositionRequest.withPosition(IntakeConstants.DEPLOYED_POSITION_ROTATIONS));
        leftIntakeMotor.setControl(
            intakePositionRequest.withPosition(IntakeConstants.LEFT_DEPLOYED_POSITION_ROTATIONS));
        deployed = true;
        agitating = false;
    }

    public void stow() {
        rightIntakeMotor.setControl(
            intakePositionRequest.withPosition(IntakeConstants.STOWED_POSITION_ROTATIONS));
        leftIntakeMotor.setControl(intakePositionRequest.withPosition(IntakeConstants.STOWED_POSITION_ROTATIONS));
    }

    public void agitate(){
        timer.reset();
        agitating = true;
        deployed = false;
    }

    public boolean isDeployed() {
        return deployed;
        //isDeployed
    }

    public boolean isAgitating() {
        return agitating;
    }
}