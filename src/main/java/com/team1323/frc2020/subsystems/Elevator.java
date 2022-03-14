// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Elevator extends Subsystem {
    LazyTalonFX motor;
    //DutyCycle absoluteEncoder;


    double targetHeight = 0;
    boolean onTarget = false;
    private static Elevator instance = null;
    public static Elevator getInstance() {
        if(instance == null)
            instance = new Elevator();
        return instance;
    }


    public Elevator() {
        motor = new LazyTalonFX(Ports.ELEVATOR, "main");

        //absoluteEncoder = new DutyCycle(new DigitalInput(Ports.ELEVATOR_ENCODER));

        motor.configForwardSoftLimitThreshold(inchesToEncUnits(Constants.Elevator.kMaxControlHeight), Constants.kCANTimeoutMs);
        motor.configReverseSoftLimitThreshold(inchesToEncUnits(Constants.Elevator.kMinControlHeight), Constants.kCANTimeoutMs);
        enableLimits(false);

        motor.config_kP(0, Constants.Elevator.kP);
        motor.config_kI(0, Constants.Elevator.kI);
        motor.config_kD(0, Constants.Elevator.kD);
        motor.config_kF(0, Constants.Elevator.kF);

        motor.setSelectedSensorPosition(inchesToEncUnits(Constants.Elevator.kStartingHeight));
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void enableLimits(boolean enable) {
        motor.configForwardSoftLimitEnable(enable, Constants.kCANTimeoutMs);
        motor.configReverseSoftLimitEnable(enable, Constants.kCANTimeoutMs);
    }

    public void configForAssent() {
        
    }
    PeriodicIO periodicIO = new PeriodicIO();

    public enum State {
        OFF, POSITION, LOCKED, OPEN_LOOP;
    }

    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State desiredState) {
        currentState = desiredState;
    }

    public double getAbsoluteEncoderPosition() {
        return 0;//-absoluteEncoder.getOutput();
    }

    public double encUnitsToInches(double encUnits) {
        return (encUnits / Constants.Elevator.kFalconTicksPerInch);
    }

    public double inchesToEncUnits(double inches) {
        return (inches * Constants.Elevator.kFalconTicksPerInch);
    }

    public double absoluteEncoderRotationsToInches(double rotations) {
        return encUnitsToInches(rotations * Constants.Elevator.kFalconToMagEncoderRatio * 2048.0);
    }

    public void resetToAbsolutePosition() {
        /*
        double cancoderOffset = getAbsoluteEncoderPosition() - Constants.Elevator.kMagEncoderStartingPosition;
        double absoluteElevatorHeight = Constants.Elevator.kStartingHeight + absoluteEncoderRotationsToInches(cancoderOffset);
        if (absoluteElevatorHeight > Constants.Elevator.kMaxInitialHeight) {
            cancoderOffset -= 1.0;
            absoluteElevatorHeight = Constants.Elevator.kStartingHeight + absoluteEncoderRotationsToInches(cancoderOffset);
        } else if (absoluteElevatorHeight < Constants.Elevator.kMinInitialHeight) {
            cancoderOffset += 1.0;
            absoluteElevatorHeight = Constants.Elevator.kStartingHeight + absoluteEncoderRotationsToInches(cancoderOffset);
        }

        if (absoluteElevatorHeight > Constants.Elevator.kMaxInitialHeight || absoluteElevatorHeight < Constants.Elevator.kMinInitialHeight) {
            DriverStation.reportError("Elevator height is out of bounds", false);
        }

        motor.setSelectedSensorPosition(inchesToEncUnits(absoluteElevatorHeight), 0, Constants.kCANTimeoutMs);
        */
        //resetToAbsolute();
    }

    private boolean elevatorPowered = false;
    
    public void setTargetHeight(double heightInches) {
        setState(State.POSITION);
        heightInches = Util.limit(heightInches, Constants.Elevator.kMinControlHeight, Constants.Elevator.kMaxControlHeight);
        periodicIO.demand = inchesToEncUnits(heightInches);
        onTarget = false;
        targetHeight = heightInches;
    }
    public void lockElevatorHeight() {
        setState(State.POSITION);
        targetHeight = encUnitsToInches(periodicIO.position);
        periodicIO.demand = periodicIO.position;
    }
    public void setOpenLoop(double demand) {
        setState(State.OPEN_LOOP);
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
        periodicIO.position = motor.getSelectedSensorPosition(0);
        periodicIO.current = motor.getOutputCurrent();
    }
    @Override
    public void writePeriodicOutputs() {
        if(currentState == State.LOCKED || currentState == State.POSITION) {
            motor.set(ControlMode.MotionMagic, periodicIO.demand);
        } else if(currentState == State.OPEN_LOOP) {
            motor.set(ControlMode.PercentOutput, periodicIO.demand);
        }
    }

    public void resetToAbsolute() {
        motor.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
    }
    @Override
    public void outputTelemetry() {
        if(false) {
            SmartDashboard.putNumber("Elevator Demand", periodicIO.demand);
            SmartDashboard.putString("Elevator State", currentState.toString());
            SmartDashboard.putNumber("Elevator Absolute Encoder", getAbsoluteEncoderPosition());
            SmartDashboard.putNumber("Elevator Height", encUnitsToInches(periodicIO.position));
            SmartDashboard.putNumber("Elevator Absolute Height", encUnitsToInches(inchesToEncUnits(absoluteEncoderRotationsToInches(getAbsoluteEncoderPosition() - Constants.Elevator.kMagEncoderStartingPosition))));
        }
    }

    @Override
    public void stop() {
        motor.setNeutralMode(NeutralMode.Coast);
    }
    public static class PeriodicIO {
        public double position = 0;
        public double current = 0;
        public double voltage = 0;

        public double demand = 0;
    }
}
