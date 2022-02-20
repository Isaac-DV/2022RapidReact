// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import java.lang.reflect.Array;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;

/** The subsystem that decides where to nuke the balls */
public class BallSplitter extends Subsystem {
    RobotState robotState;

    LazyTalonFX splitter;

    private boolean autoRotateSwerve = false;
    
    private static BallSplitter instance = null;
    public static BallSplitter getInstance() {
        if(instance == null) 
            instance = new BallSplitter();
        return instance;
    }
    public LazyTalonFX getTalon() {
        return splitter;
    }
    public BallSplitter() {
        robotState = RobotState.getInstance();

        splitter = new LazyTalonFX(Ports.BALL_SPLITTER, "main");

        splitter.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        splitter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs); 
        splitter.enableVoltageCompensation(true);
        splitter.setNeutralMode(NeutralMode.Brake);
        splitter.setInverted(TalonFXInvertType.CounterClockwise);
    }


    public enum EjectLocations {
        TEAM_HANGER(1, new Translation2d()), TEAM_TERMINAL(2, new Translation2d()), 
        OPPOSITE_HANGER(3, new Translation2d()), OPPOSITE_TERMINAL(4, new Translation2d());
        int priority;//The lower the number, the higher the priority
        Translation2d location;
        EjectLocations(int priority, Translation2d location) {
            this.priority = priority;
            this.location = location;
        }
    }
    private EjectLocations bestEjectLocation = EjectLocations.TEAM_HANGER;
    public EjectLocations getBestEjectLocation() {
        return bestEjectLocation;
    }


    public enum ControlState {
        OFF(0.0), LEFT_EJECT(-1.0), RIGHT_EJECT(1.0), POWER_SHIFED(0);//Power Shifted to the Elevator
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }
    private ControlState currentState = ControlState.OFF;
    public ControlState getState() {return currentState;}
    private void setState(ControlState desiredState) {
        currentState = desiredState;
    }

    public void setOpenLoop(double demand) {
        splitter.set(ControlMode.PercentOutput, demand);
    }

    public void conformToState(ControlState desiredState) {
        conformToState(desiredState, desiredState.speed);
    }

    public void conformToState(ControlState desiredState, double outputOverride) {
        if ((!isShifted) || (desiredState == ControlState.POWER_SHIFED)) {
            setState(desiredState);
            setOpenLoop(outputOverride);
        }
    }
    
    private boolean isShifted = false;
    public void shiftPower(boolean shiftedToElevator) {
        isShifted = shiftedToElevator;
        conformToState(ControlState.POWER_SHIFED);
    }

    public DriverStation.Alliance DSAlliance;
    public void setDSAlliance(DriverStation.Alliance alliance) {
        DSAlliance = alliance;
    }

    
    public void updateBestEjectLocation() {
        boolean locationFound = false;
        double lowestLocationsMagnitude = Double.POSITIVE_INFINITY;
        EjectLocations closestLocation = EjectLocations.TEAM_HANGER;
        for(EjectLocations ejectLocation : EjectLocations.values()) {
            double locationMagnitude = ejectLocation.location.norm();
            if(locationMagnitude < Constants.BallSplitter.kEjectorLength && !locationFound) {
                bestEjectLocation = ejectLocation;
                locationFound = true;
            }
            if(locationMagnitude < lowestLocationsMagnitude) {
                closestLocation = ejectLocation;
            }

        }
        if(!locationFound) {
            bestEjectLocation = closestLocation;//If the locations cannot be reached, set the closest
        }
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            updateBestEjectLocation();        
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };

    public Request stateRequest(ControlState desiredState) {
        return new Request() {

            @Override
            public void act() {
                conformToState(desiredState);                
            }
            
        };
    }
    /**
     * Rotates the swerve 
     * @param setAutoRotate
     * @return
     */
    public Request swerveAutoRotateRequest(boolean setAutoRotate) {
        return new Request() {
            @Override
            public void act() {
                autoRotateSwerve = setAutoRotate;
            }
        };
    }
    @Override
    public void outputTelemetry() {
        
    }
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }


}
