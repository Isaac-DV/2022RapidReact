// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team254.drivers.LazyTalonFX;

/** Add your docs here. */
public class BallEjector extends Subsystem {

    LazyTalonFX ejector;

    private static BallEjector instance = null;
    public static BallEjector getInstance() {
        if (instance == null) 
            instance = new BallEjector();
        return instance;
    }
    
    public BallEjector() {
        ejector = new LazyTalonFX(Ports.BALL_EJECTOR, "main");

        ejector.setNeutralMode(NeutralMode.Brake);
        ejector.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        ejector.enableVoltageCompensation(true);
        ejector.setInverted(TalonFXInvertType.Clockwise);
    }

    public enum ControlState {
        OFF(0.0), EJECT(0.75);/* LEFT_EJECT(0.75), RIGHT_EJECT(-0.75);*/
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }

    private ControlState currentState = ControlState.OFF;
    public ControlState getState() {
        return currentState;
    }
    public void setState(ControlState desiredState) {
        currentState = desiredState;
    }

    public void setOpenLoop(double demand) {
        ejector.set(ControlMode.PercentOutput, demand);
    }
    public void conformToState(ControlState desiredState) {
        conformToState(desiredState, desiredState.speed);
    }
    public void conformToState(ControlState desiredState, double demand) {
        setState(desiredState);
        setOpenLoop(demand);
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
        ejector.set(ControlMode.PercentOutput, 0.0);
    }

}
