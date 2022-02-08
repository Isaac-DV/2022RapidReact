// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.BallSplitter.ControlState;
import com.team254.drivers.LazyTalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Column extends Subsystem {

    BallFeeder ballFeeder;

    Solenoid PTOShifter;
    private static Column instance = null;
    public static Column getInstance() {
        if (instance == null)
            instance = new Column();
        return instance;
    }
    public Column() {
        ballFeeder = BallFeeder.getInstance();

        PTOShifter = new Solenoid(Ports.PCM, PneumaticsModuleType.REVPH , Ports.FEEDER_SHIFTER);

    }

    public enum ControlState {
        DISENGAGED(false), ENGAGED(true);
        boolean isEngaged;
        ControlState(boolean isEngaged) {
            this.isEngaged = isEngaged;
        }
    }
    private ControlState currentState = ControlState.DISENGAGED;
    public ControlState getState() {
        return currentState;
    }
    public void setControlState(ControlState desiredState) {
        currentState = desiredState;
        shiftPower(desiredState.isEngaged);
    }

    private boolean rollersPowered = false;
    private void shiftPower(boolean shiftToRollers) {
        rollersPowered = shiftToRollers;
        ballFeeder.shiftPower(shiftToRollers);
        PTOShifter.set(shiftToRollers);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Column Shifter Status", rollersPowered);
    }

    @Override
    public void stop() {
        
    }

}
