// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Column extends Subsystem {
    LazyTalonFX column;
    DigitalInput banner;
    private static Column instance = null;
    public static Column getInstance() {
        if (instance == null)
            instance = new Column();
        return instance;
    }
    public Column() {
        column = new LazyTalonFX(Ports.COLUMN, "main");
        banner = new DigitalInput(Ports.COLUMN_BANNER);

        column.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        column.enableVoltageCompensation(true);
        column.setInverted(TalonFXInvertType.Clockwise);
        column.setNeutralMode(NeutralMode.Brake);
        
    }
    public boolean getBanner() {
        return banner.get();
    }

    public enum ControlState {
        OFF(0.0), FEED_BALLS(Constants.Column.kFeedBallSpeed), EJECT(Constants.Column.kReverseSpeed), 
        INDEX_BALLS(Constants.Column.kFeedBallSpeed);
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }
    private ControlState currentState = ControlState.OFF;
    public ControlState getState() {
        return currentState;
    }
    private void setState(ControlState desiredState) {
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
            if(getBanner() && getState() == ControlState.INDEX_BALLS) {
                conformToState(ControlState.OFF);
            } else if(!getBanner() && getState() == ControlState.INDEX_BALLS) {
                conformToState(ControlState.INDEX_BALLS);
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
