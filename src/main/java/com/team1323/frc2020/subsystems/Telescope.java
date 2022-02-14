// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;


import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.Util;


import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Telescope extends Subsystem {

    Solenoid pistonRod;

    boolean rodReleased = false;
    private static Telescope instance = null;
    public static Telescope getInstance() {
        if (instance == null)
            instance = new Telescope();
        return instance;
    }
    
    public Telescope() {
        pistonRod = new Solenoid(Ports.PCM, PneumaticsModuleType.REVPH, Ports.TELESCOPE);
    }

    public enum ControlState {
        OFF(true), RELEASED(false);
        boolean isExtended;
        ControlState(boolean extendState) {
            this.isExtended = extendState;
        }
    }
    private ControlState currentState = ControlState.OFF;
    public ControlState getState() {
        return currentState;
    }
    public void setState(ControlState desiredState) {
        currentState = desiredState;
    }
    public void conformToState(ControlState desiredState) {
        setState(desiredState);
        releaseRod(desiredState.isExtended);
    }
    public void releaseRod(boolean releaseRod) {
        rodReleased = releaseRod;
        pistonRod.set(releaseRod);
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
        
    }


}
