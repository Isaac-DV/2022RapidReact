// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ClawWrist extends Subsystem {

    LazyTalonFX wrist;
    CANCoder encoder;

    Solenoid claw;

    boolean isClawOpened = true;
    double targetAngle = 0.0;
    private static ClawWrist instance = null;
    public static ClawWrist getInstance() {
        if(instance == null)
            instance = new ClawWrist();
        return instance;
    }

    public ClawWrist() {
        wrist = new LazyTalonFX(Ports.HANGER_WRIST);
        encoder = new CANCoder(Ports.HANGER_WRIST_ENCODER, "main");

        claw = new Solenoid(Ports.PCM, PneumaticsModuleType.REVPH, Ports.CLAW);

        wrist.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        wrist.enableVoltageCompensation(true);

        wrist.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        wrist.configForwardSoftLimitThreshold(degreesToEncUnits(Constants.ClawWrist.kMaxControlAngle), Constants.kCANTimeoutMs);
        wrist.configReverseSoftLimitThreshold(degreesToEncUnits(Constants.ClawWrist.kMinControlAngle), Constants.kCANTimeoutMs);
        wrist.configForwardSoftLimitEnable(true);
        wrist.configReverseSoftLimitEnable(true);


        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
        wrist.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);

        wrist.setNeutralMode(NeutralMode.Brake);

        setCurrentLimit(40);

    }

    public double getAbsolutePosition() {
        return encoder.getAbsolutePosition();
    }

    public void setCurrentLimit(double amps) {
        SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, amps, amps, 0.1);
        wrist.configSupplyCurrentLimit(currentLimit);
    }

    PeriodicIO periodicIO = new PeriodicIO();

    public enum State {
        OFF, OPEN_LOOP, POSITION, LOCKED
    }
    private State currentState = State.OFF;
    public State getState() {
        return currentState;
    }
    public void setState(State desiredState) {
        currentState = desiredState;
    }


    public void openClaw(boolean openClaw) {
        isClawOpened = openClaw;
        claw.set(openClaw);
    }

    public void setOpenLoop(double demand) {
        setState(State.OPEN_LOOP);
        periodicIO.demand = demand;
    }
    public void setWristAngle(double degrees) {
        degrees = Util.limit(degrees, Constants.ClawWrist.kMinControlAngle, Constants.ClawWrist.kMaxControlAngle);
        targetAngle = degrees;
        setState(State.POSITION);
        periodicIO.demand = degreesToEncUnits(degrees);
    }
    
    public void lockWrist() {
        setState(State.LOCKED);
        targetAngle = encUnitsToDegrees(periodicIO.position);
        periodicIO.demand = periodicIO.position;
    }

    public double encUnitsToDegrees(double encUnits) {
        return ((encUnits / 2048.0) / 360.0) / Constants.ClawWrist.kFalconToWristRatio;
    }
    public double degreesToEncUnits(double degrees) {
        return ((degrees / 360) * 2048.0 * Constants.ClawWrist.kFalconToWristRatio);
    }
    public boolean hasReachedTargetAngle() {
        if(currentState == State.POSITION || currentState == State.LOCKED) {
            if(Math.abs(targetAngle - encUnitsToDegrees(periodicIO.position)) > 
                Constants.ClawWrist.kWristAngleTolerance) {
                return true;
            }
            return false;
        }
        return false;
    }

    public Request setAngleRequst(double angle) {
        return new Request() {
            @Override
            public void act() {
                setWristAngle(angle);
            }
        };
    }
    public Request openClawRequest(boolean open) {
        return new Request() {
            @Override
            public void act() {
                openClaw(open);
            }
        };
    }


    @Override
    public void readPeriodicInputs() {
        periodicIO.position = wrist.getSelectedSensorPosition(0);
        periodicIO.current = wrist.getOutputCurrent();
        periodicIO.voltage = wrist.getMotorOutputVoltage();
    }

    @Override
    public void writePeriodicOutputs() {
        if(getState() == State.POSITION || getState() == State.LOCKED) {
            wrist.set(ControlMode.MotionMagic, periodicIO.demand);
        } else if (getState() == State.OPEN_LOOP) {
            wrist.set(ControlMode.PercentOutput, periodicIO.demand);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hanger Wrist Angle", encUnitsToDegrees(periodicIO.position));
    }

    @Override
    public void stop() {
        
    }

    public static class PeriodicIO {
        public double position = 0.0;
        public double current = 0.0;
        public double voltage = 0.0;

        public double demand = 0.0;
    }
}
