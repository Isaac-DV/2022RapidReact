// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.BallEjector.ControlState;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team254.drivers.LazyTalonFX;

/** Add your docs here. */
public class Intake extends Subsystem {
    LazyTalonFX intake;
    
    private static Intake instance = null;
    public static Intake getInstance() {
        if(instance == null)
            instance = new Intake();
        return instance;
    }

    public Intake() {
        intake = new LazyTalonFX(Ports.INTAKE, "main");
        intake.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        intake.enableVoltageCompensation(true);
        intake.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
        intake.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);
        intake.setInverted(TalonFXInvertType.Clockwise);
        intake.configOpenloopRamp(0.1, Constants.kCANTimeoutMs);
    }

    public enum ControlState {
        OFF(0.0), INTAKE(Constants.Intake.kIntakeSpeed), EJECT(-0.25);
        double speed;
        ControlState(double speed){
            this.speed = speed;
        }
    }
    
    private ControlState currentState = ControlState.OFF;
    public void setState(ControlState desiredState) {
        currentState = desiredState;
    }
    public ControlState getState() {
        return currentState;
    }

    public void conformToState(ControlState desiredState) {
        conformToState(desiredState, desiredState.speed);
    }
    public void conformToState(ControlState desiredState, double outputOverride) {
        setState(desiredState);
        setOpenLoop(outputOverride);
    }

    public void setOpenLoop(double demand) {
        intake.set(ControlMode.PercentOutput, demand);
    }

    public Request stateRequest(ControlState desiredState) {
        return new Request() {

            @Override
            public void act() {
                conformToState(desiredState);     
            }
            
        };
    }

    @Override
    public void outputTelemetry() {
        
    }

    @Override
    public void stop() {
        conformToState(ControlState.OFF);
    }
    
}
