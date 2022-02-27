// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;


import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Column extends Subsystem {
    Shooter shooter;

    LazyTalonFX column;
    DigitalInput banner;

    boolean detectedBall = false;
    private static Column instance = null;
    public static Column getInstance() {
        if (instance == null)
            instance = new Column();
        return instance;
    }
    public Column() {
        shooter = Shooter.getInstance();

        column = new LazyTalonFX(Ports.COLUMN, "main");
        banner = new DigitalInput(Ports.COLUMN_BANNER);

        column.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        column.enableVoltageCompensation(true);
        column.setInverted(TalonFXInvertType.Clockwise);
        column.setNeutralMode(NeutralMode.Brake);
        column.configOpenloopRamp(0.1, Constants.kCANTimeoutMs);

        AsynchronousInterrupt interrupt = new AsynchronousInterrupt(banner, new BiConsumer<Boolean,Boolean>() {

            @Override
            public void accept(Boolean arg0, Boolean arg1) {
                if(getState() == ControlState.INDEX_BALLS) {
                    if(getBanner()) {
                        column.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);
                        conformToState(ControlState.OFF);
                        detectedBall = true;
                    } else {
                        detectedBall = false;
                    }
                }
            }
            
        });
        interrupt.setInterruptEdges(true, true);
        interrupt.enable();
    }
    public boolean getBanner() {
        return banner.get();
    }

    public enum ControlState {
        OFF(0.0), FEED_BALLS(Constants.Column.kFeedBallSpeed), EJECT(Constants.Column.kReverseSpeed), 
        INDEX_BALLS(0.5), INTAKE(Constants.Column.kFeedBallSpeed);
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
        column.set(ControlMode.PercentOutput, demand);
    }
    public void conformToState(ControlState desiredState, double demand) {
        setState(desiredState);
        setOpenLoop(demand);
    }
    public void conformToState(ControlState desiredState) {
        conformToState(desiredState, desiredState.speed);
    }
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            
        }

        @Override
        public void onLoop(double timestamp) {
            switch(currentState) {
                case FEED_BALLS:
                    if(shooter.hasReachedSetpoint()) {
                        if(!detectedBall) {
                            setState(ControlState.INDEX_BALLS);
                        }
                        setOpenLoop(Constants.Column.kFeedBallSpeed);
                    } else {
                        setOpenLoop(0.0);
                    }
                    break;
                case INDEX_BALLS:
                    if(!getBanner()){
                        detectedBall = false;
                    }
                    if(!getBanner() && !detectedBall) {
                        column.configOpenloopRamp(0.1, Constants.kCANTimeoutMs);
                        conformToState(ControlState.INDEX_BALLS);
                    }
                    break;
                default:
                break;
            }

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
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Column Banner Sensor", getBanner());
        SmartDashboard.putString("Column State", getState().toString());
    }
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    @Override
    public void stop() {
        conformToState(ControlState.OFF);
    }

}
