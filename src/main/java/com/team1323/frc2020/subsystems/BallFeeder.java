// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Controls the ball ejector and feeder motor*/
public class BallFeeder extends Subsystem {
    BallEjector ballEjector;
    BallSplitter ballSplitter;
    
    
    LazyTalonFX feeder;

    DigitalInput banner, colorSensor;

    double feederStartTimestamp = Double.POSITIVE_INFINITY;
    double ejectorStartTimestamp = Double.POSITIVE_INFINITY;


    private static BallFeeder instance = null;
    public static BallFeeder getInstance() {
        if (instance == null)
            instance = new BallFeeder();
        return instance;
    }

    public BallFeeder() {
        ballEjector = BallEjector.getInstance();
        ballSplitter = BallSplitter.getInstance();

        feeder = new LazyTalonFX(Ports.BALL_FEEDER, "main");

        feeder.setNeutralMode(NeutralMode.Brake);
        feeder.configVoltageCompSaturation(12, Constants.kCANTimeoutMs);
        feeder.enableVoltageCompensation(true);
        feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
        feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);
        feeder.setInverted(TalonFXInvertType.Clockwise);

        banner = new DigitalInput(Ports.FEEDER_BANNER);
        colorSensor = new DigitalInput(Ports.COLOR_SENSOR);

    }

    
    public enum Ball {
        None, Blue, Red
    }
    public Ball DetectedBall = Ball.None;
    public DriverStation.Alliance DSAlliance;
    public void setDSAlliance(DriverStation.Alliance alliance) {
        DSAlliance = alliance;
    }
    public void updateDetectedBall() {
        if(banner.get() && isColorSensorRed()) { //detected a red ball
            DetectedBall = Ball.Red;
        } else if(banner.get() && !isColorSensorRed()) { //detected a blue ball
            DetectedBall = Ball.Blue;
        } else if(!banner.get()) { //does not detect a ball
            DetectedBall = Ball.None;
        }
    }

    public enum State {
        OFF, DETECT, HOLD, HOLD_DETECT, OPEN_LOOP
    }
    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State desiredState) {
        currentState = desiredState;
    }

    public void setFeederOpenLoop(double demand) {
        feeder.set(ControlMode.PercentOutput, demand);
    }
    public void setOpenLoopState(double demand) {
        setState(State.OPEN_LOOP);
        setFeederOpenLoop(demand);
    }
    public boolean isBannerSensorConnected() {
        return true;
    }
    public boolean isColorSensorRed() {
        return !colorSensor.get();
    }
    

    private boolean rollersShifted = false;
    public void shiftPower(boolean shiftToRollers) {
        rollersShifted = shiftToRollers;
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            setState(State.OFF);
            setDSAlliance(DriverStation.Alliance.Red/*DriverStation.getAlliance()*/);
            ballSplitter.setDSAlliance(DriverStation.getAlliance());
        }

        @Override
        public void onLoop(double timestamp) {
            updateDetectedBall();
            if(rollersShifted) {
                setState(State.HOLD_DETECT);
            }
            switch(currentState) {
                case OFF:
                    setFeederOpenLoop(0.0);
                    break;
                case DETECT:
                    if(DSAlliance.toString() == DetectedBall.toString()) {//Detected ball is in our favor
                        ballEjector.conformToState(BallEjector.ControlState.OFF);
                        setFeederOpenLoop(-0.25);
                    } else if(DetectedBall != Ball.None) {//Detected opponents ball
                        //ballSplitter.fieldRelativeEject(timestamp);
                        setFeederOpenLoop(Constants.Intake.kIntakeSpeed);
                        ballSplitter.conformToState(BallSplitter.ControlState.RIGHT_EJECT);
                        if(ejectorStartTimestamp == Double.POSITIVE_INFINITY) {
                            ballEjector.conformToState(BallEjector.ControlState.EJECT);
                            ejectorStartTimestamp = timestamp;
                        }
                    } else if (DetectedBall == Ball.None) {
                        ballSplitter.conformToState(BallSplitter.ControlState.OFF);
                        setFeederOpenLoop(Constants.Intake.kIntakeSpeed);
                    }
                    break;
                case HOLD:
                    ballSplitter.conformToState(BallSplitter.ControlState.OFF);
                    //setFeederOpenLoop(1.0);
                    break;
                case HOLD_DETECT:
                    if(DetectedBall != Ball.None) {//Detected opponents ball
                        ballSplitter.fieldRelativeEject(timestamp);
                    }
                    //setFeederOpenLoop(1.0);
                    break;
                default:
                    break;
            }
            
        
            if((timestamp - ejectorStartTimestamp) > 1.5) {
                ballEjector.conformToState(BallEjector.ControlState.OFF);
                ejectorStartTimestamp = Double.POSITIVE_INFINITY;
            }
            
        }

        @Override
        public void onStop(double timestamp) {
            
        }
        
    };

    public Request stateRequest(State desiredState) {
        return new Request() {
            @Override
            public void act() {
                setState(desiredState);
            }
            
        };
    }
    public Request openLoopRequest(double speed) {
        return new Request() {
            @Override
            public void act() {
                setOpenLoopState(speed);
            }
        };
    }
    @Override
    public void outputTelemetry() {
        //SmartDashboard.putString("Ball Feeder State", getState().toString()); 
        SmartDashboard.putBoolean("Ball Feeder Banner Sensor", banner.get());
        SmartDashboard.putBoolean("Ball Color Sensor", isColorSensorRed());
        SmartDashboard.putString("Detected Ball", DetectedBall.toString());   
    }


    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
      enabledLooper.register(loop);
    }
    @Override
    public void stop() {
        feeder.set(ControlMode.PercentOutput, 0.0);
    }
}