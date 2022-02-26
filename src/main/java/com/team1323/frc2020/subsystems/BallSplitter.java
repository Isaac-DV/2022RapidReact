// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.Util;
import com.team254.drivers.LazyTalonFX;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** The subsystem that decides where to nuke the balls */
public class BallSplitter extends Subsystem {
    RobotState robotState;

    LazyTalonFX splitter;
    Swerve swerve;

    private boolean autoRotateSwerve = false;
    private double targetSwerveTheta = 0;
    
    Translation2d translationVector = new Translation2d();

    private static BallSplitter instance = null;
    public static BallSplitter getInstance() {
        if(instance == null) 
            instance = new BallSplitter();
        return instance;
    }
    public LazyTalonFX getTalon() {
        return splitter;
    }
    public BallSplitter() {
        robotState = RobotState.getInstance();
        swerve = Swerve.getInstance();
        splitter = new LazyTalonFX(Ports.BALL_SPLITTER, "main");

        splitter.configVoltageCompSaturation(12.0, Constants.kCANTimeoutMs);
        splitter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.kCANTimeoutMs); 
        splitter.enableVoltageCompensation(true);
        splitter.setNeutralMode(NeutralMode.Brake);
        splitter.setInverted(TalonFXInvertType.Clockwise);
        splitter.configPeakOutputReverse(-1);
        splitter.configPeakOutputForward(1);
        splitter.configOpenloopRamp(0.0, Constants.kCANTimeoutMs);
    }


    public enum EjectLocations {
        TEAM_HANGER(1, Constants.kBottomLeftQuadrantPose.getTranslation()), 
        TEAM_TERMINAL(2, Constants.kBottomRightQuadrantPose.getTranslation()), 
        OPPOSITE_HANGER(3, Constants.kTopRightQuadrantPose.getTranslation());
        int priority;//The lower the number, the higher the priority
        Translation2d location;
        EjectLocations(int priority, Translation2d location) {
            this.priority = priority;
            this.location = location;
        }
    }
    EjectLocations[] ejectLocationsArray = {EjectLocations.TEAM_HANGER, EjectLocations.TEAM_TERMINAL, EjectLocations.OPPOSITE_HANGER};
    private EjectLocations bestEjectLocation = EjectLocations.TEAM_HANGER;
    public EjectLocations getBestEjectLocation() {
        return bestEjectLocation;
    }


    public enum ControlState {
        OFF(0.0), LEFT_EJECT(-1.0), RIGHT_EJECT(1.0);//Power Shifted to the Elevator
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

    ControlState bestSplitterState = ControlState.OFF; //Determines the best direction for the splitter motor to go in
                                                      //based on the robots position on the field

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

    
    public void updateBestEjectLocation() {
        boolean locationFound = false;
        double lowestLocationsMagnitude = Double.POSITIVE_INFINITY;
        EjectLocations closestLocation = EjectLocations.TEAM_HANGER;
        for(EjectLocations ejectLocation : ejectLocationsArray) {
            double locationMagnitude = ejectLocation.location.translateBy(swerve.pose.getTranslation().inverse()).norm();
            if(locationMagnitude < Constants.BallSplitter.kEjectorLength && !locationFound) {
                bestEjectLocation = ejectLocation;
                locationFound = true;
            }
            if(locationMagnitude < lowestLocationsMagnitude) {
                lowestLocationsMagnitude = locationMagnitude;
                closestLocation = ejectLocation;
            }
        }
        if(!locationFound) {
            bestEjectLocation = closestLocation;//If the locations cannot be reached, set the closest
        }
    }
    public void updateRobotToClosestCorner() {
        Pose2d robotPose = swerve.pose;
        Translation2d positionVector = bestEjectLocation.location;
        Rotation2d robotToLocationTheta = positionVector.direction();
        Rotation2d robotRotation = robotPose.getRotation();
        Rotation2d leftSideEjectDelta = robotToLocationTheta.rotateBy(robotRotation.inverse().
                                        rotateBy(new Rotation2d(90).fromDegrees(90)).inverse());
        Rotation2d rightSideEjectDelta = robotToLocationTheta.rotateBy(robotRotation.inverse().
                                        rotateBy(new Rotation2d().fromDegrees(-90)).inverse());
        if(Math.abs(rightSideEjectDelta.getDegrees()) < Math.abs(leftSideEjectDelta.getDegrees())) { //The right side is closer
            robotToLocationTheta.rotateBy(new Rotation2d().fromDegrees(90));
        } else if(Math.abs(rightSideEjectDelta.getDegrees()) > Math.abs(leftSideEjectDelta.getDegrees())) {//The left side is closer
            robotToLocationTheta.rotateBy(new Rotation2d().fromDegrees(-90));
        }
        robotToLocationTheta.inverse();
        if(targetSwerveTheta < 0) {
            targetSwerveTheta -= 90;
        } else if(targetSwerveTheta > 0) {
            targetSwerveTheta += 90;
        }

        double swerveNeg180to180Rotation = Util.boundAngleNeg180to180Degrees(robotRotation.getDegrees());
        if((-180 < swerveNeg180to180Rotation) && (swerveNeg180to180Rotation < 0)) {
            bestSplitterState = ControlState.LEFT_EJECT;
        } else if((0 < swerveNeg180to180Rotation) && (swerveNeg180to180Rotation < 180)) {
            bestSplitterState = ControlState.RIGHT_EJECT;
        }
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
           updateBestEjectLocation();
           updateRobotToClosestCorner();
            
        }

        @Override
        public void onStop(double timestamp) {

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
    /**
     * Rotates the swerve 
     * @param setAutoRotate
     * @return
     */
    public Request swerveAutoRotateRequest(boolean setAutoRotate) {
        return new Request() {
            @Override
            public void act() {
                autoRotateSwerve = setAutoRotate;
            }
        };
    }
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Swerve Rotation to BEL", targetSwerveTheta);
        SmartDashboard.putNumber("Swerve Raw Theta",  Util.boundAngle0to360Degrees(translationVector.direction().getDegrees()));
        SmartDashboard.putString("Eject Mode", bestSplitterState.toString());
        /*SmartDashboard.putString("Closest Oof Location", bestEjectLocation.toString());
        SmartDashboard.putNumber("Closest Oof Location magnitude", bestEjectLocation.location.translateBy(swerve.pose.getTranslation().inverse()).norm());
        SmartDashboard.putNumber("TopRight Magnitude", EjectLocations.OPPOSITE_HANGER.location.getTranslation().translateBy(swerve.pose.getTranslation().inverse()).norm());
        SmartDashboard.putNumber("BotLeft Magnitude", EjectLocations.TEAM_HANGER.location.getTranslation().translateBy(swerve.pose.getTranslation().inverse()).norm());
        SmartDashboard.putNumber("BotRight Magnitude", EjectLocations.TEAM_TERMINAL.location.getTranslation().translateBy(swerve.pose.getTranslation().inverse()).norm());*/

        
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
