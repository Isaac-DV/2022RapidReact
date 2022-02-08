// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class BallSplitter extends Subsystem {
    RobotState robotState;

    LazyTalonFX splitter;

    private static BallSplitter instance = null;
    public static BallSplitter getInstance() {
        if(instance == null) 
            instance = new BallSplitter();
        return instance;
    }
    public BallSplitter() {
        robotState = RobotState.getInstance();

        splitter = new LazyTalonFX(Ports.BALL_SPLITTER);
        splitter.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        splitter.enableVoltageCompensation(true);
        splitter.setNeutralMode(NeutralMode.Brake);
        splitter.setInverted(TalonFXInvertType.Clockwise);
    }

    public enum ControlState {
        OFF(0.0), LEFT_EJECT(-0.5), RIGHT_EJECT(0.5);
        double speed;
        ControlState(double speed) {
            this.speed = speed;
        }
    }
    private ControlState currentState = ControlState.OFF;
    public ControlState getState() {return currentState;}
    private void setState(ControlState desiredState) {
        currentState = desiredState;
    }
    public void setOpenLoop(double demand) {
        splitter.set(ControlMode.PercentOutput, demand);
    }
    public void conformToState(ControlState desiredState) {
        conformToState(desiredState, desiredState.speed);
    }
    public void conformToState(ControlState desiredState, double outputOverride) {
        setState(desiredState);
        setOpenLoop(outputOverride);
    }

    public DriverStation.Alliance DSAlliance;
    public void setDSAlliance(DriverStation.Alliance alliance) {
        DSAlliance = alliance;
    }

    public void fieldRelativeEject(double timestamp) {
        Pose2d robotsPosition = robotState.getLatestFieldToVehicle().getValue();
        Translation2d closestCornerVector = new Translation2d(999.0, 999.0);
        for(int i = 0; i < Constants.kFieldCornerPositions.size(); i++) {
            Translation2d currentSelectedCorner = Constants.kFieldCornerPositions.get(i).getTranslation();
            Translation2d vehicleToCorner = currentSelectedCorner.translateBy(robotsPosition.getTranslation().inverse());
            
            if((closestCornerVector.norm()) > (vehicleToCorner.norm())) { //If the current Vector's magnitude is smaller than the smallest,
                                                                          //set that as the closest 
                   closestCornerVector = vehicleToCorner; 
            }
        }
    }

    @Override
    public void outputTelemetry() {
        
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);        
    }


}
