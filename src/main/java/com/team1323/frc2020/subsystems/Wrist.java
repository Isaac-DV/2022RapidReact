// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.Settings;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.SensorCheck;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Wrist extends Subsystem {


    LazyTalonFX wrist;
    DutyCycle encoder;
    SensorCheck encoderCheck;

    double wristTargetAngle = Constants.Wrist.kStowedAngle;
    private boolean encoderDisconected = false;
    private boolean zeroedAbsolutely = false;
    private static Wrist instance = null;
    public static Wrist getInstance() {
        if (instance==null)
            instance = new Wrist();
        return instance;
    }
    public Wrist() {
        wrist = new LazyTalonFX(Ports.WRIST, "main");
        encoder = new DutyCycle(new DigitalInput(Ports.WRIST_ENCODER));
        encoderCheck = new SensorCheck(encoder, ()->{
            encoderDisconected = true;
        });
        wrist.setInverted(TalonFXInvertType.CounterClockwise);
        wrist.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        wrist.configForwardSoftLimitThreshold((int)degreesToEncUnits(Constants.Wrist.kMaxWristAngle), Constants.kCANTimeoutMs);
        wrist.configReverseSoftLimitThreshold((int)degreesToEncUnits(Constants.Wrist.kMinWristAngle),Constants.kCANTimeoutMs);
        wrist.configForwardSoftLimitEnable(true);
        wrist.configReverseSoftLimitEnable(true);
        wrist.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        wrist.enableVoltageCompensation(true);

        wrist.setNeutralMode(NeutralMode.Coast);


        //periodicIO.demand = degreesToEncUnits(Constants.Wrist.kStowedAngle);
        configWristPID();
        resetToAbsolutePosition();
    }

    private void configWristPID() {
        wrist.selectProfileSlot(0, 0);
        wrist.config_kP(0, Constants.Wrist.kP, Constants.kCANTimeoutMs);
        wrist.config_kI(0, Constants.Wrist.kI, Constants.kCANTimeoutMs);
        wrist.config_kD(0, Constants.Wrist.kD, Constants.kCANTimeoutMs);
        wrist.config_kF(0, Constants.Wrist.kF, Constants.kCANTimeoutMs);

        wrist.config_IntegralZone(0, (int)degreesToEncUnits(5), 10);
        wrist.configMotionCruiseVelocity((int)(Constants.Wrist.kMaxSpeed * 1.0), Constants.kCANTimeoutMs);
        wrist.configMotionAcceleration((int)(Constants.Wrist.kMaxSpeed * 2.5), Constants.kCANTimeoutMs);
        wrist.configMotionSCurveStrength(0);

    }
    public double getAbsoluteEncoderDegrees() {
        return encoder.getOutput() * 360.0 * 1.0;
    }
    public enum State {
        OFF, OPEN_LOOP, POSITION, LOCK, DISABLED
    }
    
    private State currentState = State.OFF;

    public void setState(State desiredState) {
        currentState = desiredState;
    }
    public State getState() {
        return currentState;
    }

    public double getAngle() {
        return encUnitsToDegrees(periodicIO.position);
    }

    private void setAngle(double angle) {
        angle = Math.max(Math.min(angle, Constants.Wrist.kMaxWristAngle), Constants.Wrist.kMinWristAngle);
        System.out.println("Wrist Angle set to " + angle);
        wristTargetAngle = angle;
        periodicIO.demand = degreesToEncUnits(angle);
    }
    public void setWristAngle(double angle) {
        setState(State.POSITION);
        setAngle(angle);
    }
    public void setWristLocked() {
        setState(State.LOCK);
        setAngle(encUnitsToDegrees(periodicIO.position));
    }

    public void setOpenLoop(double demand) {
        periodicIO.demand = demand * 0.25;
        setState(State.OPEN_LOOP);
    }
    PeriodicIO periodicIO = new PeriodicIO();



    public double degreesToEncUnits(double degrees) {
        return (degrees / 360 * 2048.0 * Constants.Wrist.kFalconToWristRatio);
    }
    public double encUnitsToDegrees(double encUnits) {
        return ((encUnits / 2048.0) / Constants.Wrist.kFalconToWristRatio * 360.0);
    }

    public void zeroWrist() {
        zeroedAbsolutely = true;
    }

    public void resetToAbsolutePosition() {
        if(!zeroedAbsolutely) {
            double cancoderOffset = Util.boundAngle0to360Degrees(getAbsoluteEncoderDegrees() - Constants.Wrist.kWristStartingEncoderPosition);
            double absoluteWristAngle = Constants.Wrist.kWristStartingAngle + (cancoderOffset / Constants.Wrist.kCANCoderToWristRatio);
            if (absoluteWristAngle > Constants.Wrist.kMaxInitialAngle) {
                cancoderOffset -= 360.0;
                absoluteWristAngle = Constants.Wrist.kWristStartingAngle + (cancoderOffset / Constants.Wrist.kCANCoderToWristRatio);
            } else if (absoluteWristAngle < Constants.Wrist.kMinInitialAngle) {
                cancoderOffset += 360.0;
                absoluteWristAngle = Constants.Wrist.kWristStartingAngle + (cancoderOffset / Constants.Wrist.kCANCoderToWristRatio);
            }   

            if (absoluteWristAngle > Constants.Wrist.kMaxInitialAngle || absoluteWristAngle < Constants.Wrist.kMinInitialAngle) {
                DriverStation.reportError("Wrist angle is out of bounds", false);
            }

            wrist.setSelectedSensorPosition((int)degreesToEncUnits(absoluteWristAngle), 0, 0);
        }
        
    }


    public Request setWristState(State desiredState) {
        return new Request() {

            @Override
            public void act() {
                setState(desiredState);
            }

        };
    }
    public Request setWristAngleRequest(double targetAngle) {
        return new Request() {

            @Override
            public void act() {
                setWristAngle(targetAngle);
            }
            
        };
    }
    public Request setWristIntakeRequest() {
        return new Request() {

            @Override
            public void act() {
                setWristAngle(Constants.Wrist.kIntakeAngle);                
            }

            @Override
            public boolean isFinished() {
                return (getState() == State.OPEN_LOOP) || ((getState() == State.POSITION || getState() == State.LOCK) &&
                    Math.abs(encUnitsToDegrees(periodicIO.demand) - encUnitsToDegrees(periodicIO.position)) <= 10.0);
            }
        };
    }
    public Request setWristStowedRequest() {
        return new Request() {
            @Override
            public void act() {
                setWristAngle(Constants.Wrist.kStowedAngle);
            }

            @Override
            public boolean isFinished() {
                return (getState() == State.OPEN_LOOP) || ((getState() == State.POSITION || getState() == State.LOCK) &&
                    Math.abs(encUnitsToDegrees(periodicIO.demand) - encUnitsToDegrees(periodicIO.position)) <= 10.0);
            }
        };
    }
    @Override
    public void writePeriodicOutputs() {
        if(currentState == State.OPEN_LOOP) {
            wrist.set(ControlMode.PercentOutput, periodicIO.demand);
        } else if ((currentState == State.POSITION) || (currentState == State.LOCK)) {
            wrist.set(ControlMode.MotionMagic, periodicIO.demand);
        } else {
            wrist.set(ControlMode.PercentOutput, 0.0);
        }

    }
    @Override
    public void readPeriodicInputs() {
        periodicIO.position = wrist.getSelectedSensorPosition(0);
        periodicIO.velocity = wrist.getSelectedSensorVelocity(0);
        periodicIO.current = wrist.getOutputCurrent();
    }
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Wrist Falcon Position", encUnitsToDegrees(periodicIO.position));
        if(Settings.debugWrist()) {
            SmartDashboard.putNumber("Wrist Falcon Target Angle", wristTargetAngle);
            SmartDashboard.putNumber("Wrist Position Error", wristTargetAngle - encUnitsToDegrees(periodicIO.position));
            SmartDashboard.putNumber("Wrist Absolute Position", getAbsoluteEncoderDegrees());
        }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    public static class PeriodicIO {
        public double position;
        public double velocity;
        public double voltage;
        public double current;

        public double demand;
    }

}