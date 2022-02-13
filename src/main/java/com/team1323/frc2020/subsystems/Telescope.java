// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Telescope extends Subsystem {

    LazyTalonFX motor;
    Solenoid brake;

    private static Telescope instance = null;
    public static Telescope getInstance() {
        if (instance == null)
            instance = new Telescope();
        return instance;
    }

    public Telescope() {
        motor = new LazyTalonFX(Ports.TELESCOPE);
        brake = new Solenoid(Ports.PCM, PneumaticsModuleType.REVPH, Ports.TELESCOPE_BRAKE);

        motor.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        motor.enableVoltageCompensation(true);
        

        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs);
        motor.configForwardSoftLimitThreshold(0, Constants.kCANTimeoutMs);
        motor.configReverseSoftLimitThreshold(0, Constants.kCANTimeoutMs);
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);

        motor.setInverted(TalonFXInvertType.Clockwise);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 50);

        setCurrentLimit(40);
    }

    public void setCurrentLimit(int amps) {
        SupplyCurrentLimitConfiguration supplyCurrentConfig = new SupplyCurrentLimitConfiguration(true, amps, amps, 0.1);
        motor.configSupplyCurrentLimit(supplyCurrentConfig);
    }

    public void configForAscent() {


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
    private void setOpenLoop(double demand) {
        setState(State.OPEN_LOOP);
        periodicIO.demand = demand;
    }
    public int inchesToEncUnits(double inches) {
        return (int)(Constants.Telescope.kTicksPerInch * inches);
    }
    public double encUnitsToInches(double encUnits) {
        return (encUnits / Constants.Telescope.kTicksPerInch);
    }



    @Override
    public void readPeriodicInputs() {
        periodicIO.position = motor.getSelectedSensorPosition(0);
        periodicIO.velocity = motor.getSelectedSensorVelocity(0);
        periodicIO.current = motor.getOutputCurrent();
        periodicIO.voltage = motor.getMotorOutputVoltage();
    }
    @Override
    public void writePeriodicOutputs() {
        if((getState() == State.LOCKED) || (getState() == State.POSITION)) {
            motor.set(ControlMode.MotionMagic, periodicIO.demand);
        } else if(getState() == State.OPEN_LOOP) {
            motor.set(ControlMode.PercentOutput, periodicIO.demand);
        } else {
            motor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Telescope Height", encUnitsToInches(periodicIO.position));
    }

    @Override
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0.0);  
    }

    public static class PeriodicIO {
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double voltage = 0; 

        public double demand;
    }

}
