// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;


import java.util.function.BiConsumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.Settings;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.SmartTuner;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Controls the ball ejector and feeder motor*/
public class BallFeeder extends Subsystem {

    BallSplitter ballSplitter;
    Intake intake;
    Shooter shooter;
    
    LazyTalonFX feeder;

    DigitalInput banner, colorSensor;

    double intakeStartTimestamp = Double.POSITIVE_INFINITY;
    double splitterStartTimestamp = Double.POSITIVE_INFINITY;
    boolean intakeFeedEnabled = false;
    boolean isIntakeOpenLoop = false;
    boolean sentUpBall = false;
    boolean canChangeDetectedBall = true;
    boolean intakeResuckActivated = false;
    private boolean intakeResuckHasBeenActivated = false;
    public boolean hasIntakeResuckBeenActivated() {
        return intakeResuckHasBeenActivated;
    }
    public boolean hasSentUpBall() {
        return sentUpBall;
    }
    public void setSentUpBall(boolean bool) {
        sentUpBall = bool;
    }
    boolean printFeeder;
    public void setPrintFeeder(boolean enabled) {
        printFeeder = enabled;
    }
    private boolean autoDetectEnabled = true;
    public boolean isAutoDetectEnabled() {
        return autoDetectEnabled;
    }


    private double testCounter = 0;
    private int ballCounter = 0; //The amount of balls that are in the robot

    private static BallFeeder instance = null;
    public static BallFeeder getInstance() {
        if (instance == null)
            instance = new BallFeeder();
        return instance;
    }

    public BallFeeder() {
        ballSplitter = BallSplitter.getInstance();
        intake = Intake.getInstance();

        shooter = Shooter.getInstance();

        feeder = new LazyTalonFX(Ports.BALL_FEEDER, "main");

        feeder.configOpenloopRamp(0.0);
        feeder.setNeutralMode(NeutralMode.Brake);
        feeder.configVoltageCompSaturation(12, Constants.kCANTimeoutMs);
        feeder.enableVoltageCompensation(true);
        feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
        feeder.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100);
        feeder.setInverted(TalonFXInvertType.CounterClockwise);

        banner = new DigitalInput(Ports.FEEDER_BANNER);
        colorSensor = new DigitalInput(Ports.COLOR_SENSOR);

        AsynchronousInterrupt bannerInterrupt = new AsynchronousInterrupt(banner, new BiConsumer<Boolean,Boolean>() {

            @Override
            public void accept(Boolean arg0, Boolean arg1) {
                if (!banner.get()) {
                    canChangeDetectedBall = true;
                }
            }        

        });
        bannerInterrupt.setInterruptEdges(true, true);
        bannerInterrupt.enable();

        AsynchronousInterrupt colorSensorInterrupt = new AsynchronousInterrupt(colorSensor, new BiConsumer<Boolean,Boolean>() {

            @Override
            public void accept(Boolean arg0, Boolean arg1) {
                if(currentState == State.DETECT)
                    updateDetectionLogic(Timer.getFPGATimestamp());
                if (isColorSensorRed()) {
                    //System.out.println("Interrupt detected a red ball");
                }
            }        

        });
        colorSensorInterrupt.setInterruptEdges(true, true);
        colorSensorInterrupt.enable();

        initializeDashboardValues();
    }
    private void initializeDashboardValues() {
        boolean autoDetectToggle = SmartDashboard.getBoolean("Auto Detect Balls", true);
        SmartDashboard.putBoolean("Auto Detect Balls", autoDetectToggle);
    }
    public enum BallType {
        Team, Opponent, None
    }
    public BallType detectedBallType = BallType.None;
    public BallType getDetectedBallType() {
        return detectedBallType;
    }
    
    public enum Ball {
        None, Blue, Red
    }
    public Ball DetectedBall = Ball.None;
    public Ball getDetectedBall() {
        return DetectedBall;
    }
    public DriverStation.Alliance DSAlliance;
    public void setDSAlliance(DriverStation.Alliance alliance) {
        DSAlliance = alliance;
    }
    public void updateDetectedBall() {
        if(banner.get() && !isColorSensorRed()) { //detected a blue ball
            if (canChangeDetectedBall)
                DetectedBall = Ball.Blue;
        } else if(isColorSensorRed()) { //detected a red ball
            DetectedBall = Ball.Red;
            canChangeDetectedBall = false;
        } else if(!banner.get()) { //does not detect a ball
            DetectedBall = Ball.None;
            canChangeDetectedBall = true;
        }
    }

    public enum State {
        OFF, DETECT, HOLD, OPEN_LOOP, FEED_BALLS
    }
    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State desiredState) {
        if (desiredState == State.DETECT && isAutoDetectEnabled()) {
            pendingShutdown = false;
            setFeederOpenLoop(1.0);
        }
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

    private boolean pendingShutdown = false;
    public void queueShutdown(boolean shutdown) {
        pendingShutdown = shutdown;
    }

    private synchronized void updateDetectionLogic(double timestamp) {
        updateDetectedBall();

        if(DSAlliance.toString() == DetectedBall.toString()) {//Detected ball is in our favor
            if(detectedBallType != BallType.Team) {
                System.out.println(DetectedBall.toString() + " Detected" + ", Color Sensor : " + isColorSensorRed() + ", Is Feeder banner activated : " + banner.get());
            }
            detectedBallType = BallType.Team;
            setFeederOpenLoop(0);
            if(intake.getState() != Intake.ControlState.EJECT && intake.getState() != Intake.ControlState.INTAKE && intake.getLastState() != Intake.ControlState.EJECT) { //Ensures that the Intake is not in the Eject Mode
                intake.conformToState(Intake.ControlState.AUTO_FEED_INTAKE);
            }
            intakeStartTimestamp = timestamp;
            /*if(Math.abs(timestamp - splitterStartTimestamp) <= 0.5 && Double.isFinite(splitterStartTimestamp)) {
                ballSplitter.conformToState(BallSplitter.ControlState.OFF);
                splitterStartTimestamp = Double.POSITIVE_INFINITY;
            }*/
            sentUpBall = true;
            
        } else if(DetectedBall != Ball.None) {//Detected opponents ball
            if(detectedBallType != BallType.Opponent) {
                System.out.println(DetectedBall.toString() + " Detected" + ", Color Sensor : " + isColorSensorRed() + ", Is Feeder banner activated : " + banner.get());
            }
            detectedBallType = BallType.Opponent;
            setFeederOpenLoop(1.0);
            ballSplitter.conformToState(ballSplitter.bestSplitterState);
            //column.conformToState(Column.ControlState.OFF);
            splitterStartTimestamp = timestamp;
            
        } else if (DetectedBall == Ball.None) { //There's no ball detected
            if(detectedBallType != BallType.None) {
                System.out.println(DetectedBall.toString() + " Detected" + ", Color Sensor : " + isColorSensorRed() + ", Is Feeder banner activated : " + banner.get());
            }
            detectedBallType = BallType.None;
            if(pendingShutdown) {
                pendingShutdown = false;
                setFeederOpenLoop(0.0);
            }
        }
        if (Double.isFinite(splitterStartTimestamp) && (timestamp - splitterStartTimestamp) > Constants.BallFeeder.kSplitterRunTime) {
            ballSplitter.conformToState(BallSplitter.ControlState.OFF);
            if(intake.getState() == Intake.ControlState.INTAKE) {
                //column.setIndexBallState();
            }
            splitterStartTimestamp = Double.POSITIVE_INFINITY;
        }
    }
    

    private boolean rollersShifted = false;
    public void shiftPower(boolean shiftToRollers) {
        rollersShifted = shiftToRollers;
    }
    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            setState(State.DETECT);
            setDSAlliance(/*DriverStation.Alliance.Red*/DriverStation.getAlliance());
            setFeederOpenLoop(0.0);
        }

        @Override
        public void onLoop(double timestamp) {
            if(printFeeder)
                //System.out.println("Detected Ball : " + DetectedBall.toString() + ", Get Banner : " + banner.get() + ", Get Color Sensor : " + isColorSensorRed());
            if(!isAutoDetectEnabled()) {
                setState(State.OFF);
                ballSplitter.setOpenLoop(0.0);
            }
            switch(currentState) {
                case OFF:
                    setFeederOpenLoop(0.0);
                    break;
                case DETECT:
                    updateDetectionLogic(timestamp);
                    break; 
                case HOLD:
                    setFeederOpenLoop(1.0);
                    ballSplitter.setOpenLoop(0.0);
                    if(banner.get()) {
                        intakeResuckActivated = true;
                        intakeResuckHasBeenActivated = true;
                        intake.setOpenLoop(-0.25);
                    }
                    if(!banner.get() && intakeResuckActivated) {
                        intake.setOpenLoop(0.0);
                        intakeResuckActivated = false;
                    }
                default:
                    break;
            }
            if((timestamp - intakeStartTimestamp) > Constants.BallFeeder.kIntakeAutoRunTime && Double.isFinite(intakeStartTimestamp)) {
                if(intake.getState() == Intake.ControlState.AUTO_FEED_INTAKE) { //If the intake is in any other state besides the autoFeedMode, it will not disable
                    intake.conformToState(Intake.ControlState.OFF);
                }
                intakeStartTimestamp = Double.POSITIVE_INFINITY;
            }
        }

        @Override
        public void onStop(double timestamp) {
            
        }
        
    };
    public Request intakeFeedRequest() {
        return new Request() {
            @Override
            public void act() {
                setState(State.DETECT);
            }
        };
    }
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
        autoDetectEnabled = SmartDashboard.getBoolean("Auto Detect Balls", true);
        SmartDashboard.putBoolean("Ball Feeder Banner Sensor", banner.get());
        SmartDashboard.putBoolean("Ball Color Sensor", isColorSensorRed());
        SmartDashboard.putString("Ball Feeder Detected Ball", DetectedBall.toString());
        SmartDashboard.putString("Ball Feeder State", getState().toString()); 
        //SmartDashboard.putString("Selected Alliance", DSAlliance.toString());
        
        if(Settings.debugFeeder()) {
            SmartDashboard.putBoolean("Ball Feeder Banner Sensor", banner.get());
            SmartDashboard.putBoolean("Ball Color Sensor", isColorSensorRed());
            SmartDashboard.putString("Ball Feeder Detected Ball", DetectedBall.toString());
            SmartDashboard.putString("Ball Eject Location", ballSplitter.bestSplitterState.toString());
        }
        
        /*if(feeder.getBusVoltage() == 0)
            DriverStation.reportError("FEEDER MOTOR NOT DETECTED", false);*/
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