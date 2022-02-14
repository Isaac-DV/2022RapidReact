// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class Elevator extends Subsystem {
    BallSplitter ballSplitter;
    LazyTalonFX splitterMotor;

    Solenoid PTOShifter;

    double targetHeight = 0;
    boolean onTarget = false;
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if(instance == null)
            instance = new Elevator();
        return instance;
    }


    public Elevator() {
        ballSplitter = BallSplitter.getInstance();
        splitterMotor = ballSplitter.getTalon();

        PTOShifter = new Solenoid(Ports.PCM, PneumaticsModuleType.REVPH, Ports.ELEVATOR_SHIFTER);
    }

    public void configForAssent() {
        
    }
    PeriodicIO periodicIO = new PeriodicIO();

    public enum State {
        OFF(false), POSITION(true), LOCKED(true), OPEN_LOOP(true);
        boolean isEngaged;
        State(boolean engagedStatus) {
            this.isEngaged = engagedStatus;
        } 
    }

    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State desiredState) {
        currentState = desiredState;
    }

    public double encUnitsToInches(double encUnits) {
        return (encUnits / Constants.Elevator.kTicksPerInch);
    }
    public int inchesToEncUnits(double inches) {
        return (int)(inches * Constants.Elevator.kTicksPerInch);
    }

    private boolean elevatorPowered = false;
    public void shiftPower(boolean shiftedToElevator) {
        elevatorPowered = shiftedToElevator;
        PTOShifter.set(shiftedToElevator);
        ballSplitter.shiftPower(shiftedToElevator);
    }
    public void setTargetHeight(double heightInches) {
        setState(State.POSITION);
        shiftPower(State.POSITION.isEngaged);
        heightInches = Util.limit(heightInches, Constants.Elevator.kMinControlHeight, Constants.Elevator.kMaxControlHeight);
        periodicIO.demand = inchesToEncUnits(heightInches);
        onTarget = false;
        targetHeight = heightInches;
    }
    public void setOpenLoop(double demand) {
        setState(State.OPEN_LOOP);
        shiftPower(State.OPEN_LOOP.isEngaged);
        periodicIO.demand = demand;
    }

    public Request setHeightRequest(double targetHeight) {
        return new Request() {
            @Override
            public void act() {
                setTargetHeight(targetHeight);
            }
        };
    }
    public Request setOpenLoopRequest(double demand) {
        return new Request() {
            @Override
            public void act() {
                setOpenLoop(demand);
            }
        };
    }
    @Override
    public void readPeriodicInputs() {
        periodicIO.position = splitterMotor.getSelectedSensorPosition(0);
        periodicIO.current = splitterMotor.getOutputCurrent();
        periodicIO.voltage = splitterMotor.getMotorOutputVoltage();
    }
    @Override
    public void writePeriodicOutputs() {
        if(currentState == State.LOCKED || currentState == State.POSITION) {
            splitterMotor.set(ControlMode.MotionMagic, periodicIO.demand);
        } else if(currentState == State.OPEN_LOOP) {
            splitterMotor.set(ControlMode.PercentOutput, periodicIO.demand);
        }
    }

    public void resetToAbsolute() {
        splitterMotor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
    }
    @Override
    public void outputTelemetry() {
        
    }

    @Override
    public void stop() {

    }
    public static class PeriodicIO {
        public double position = 0;
        public double current = 0;
        public double voltage = 0;

        public double demand = 0;
    }
}
