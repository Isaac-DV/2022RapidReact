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
import com.team1323.frc2020.Settings;
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
    double targetVelocity = 0.0;
    
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

        splitter.selectProfileSlot(0, 0);
        splitter.config_kP(0, 0.0, Constants.kCANTimeoutMs);
        splitter.config_kI(0, 0.0, Constants.kCANTimeoutMs);
        splitter.config_kD(0, 0.0, Constants.kCANTimeoutMs);
        splitter.config_kF(0, 0.051, Constants.kCANTimeoutMs);
    }


    public enum EjectLocations {
        TEAM_HANGER(1, Constants.kBottomLeftQuadrantPose.getTranslation()), 
        TEAM_TERMINAL(2, Constants.kBottomRightQuadrantPose.getTranslation()), 
        OPPOSITE_HANGER(3, Constants.kTopRightQuadrantPose.getTranslation());
        int priority;//The lower the number, the higher the priority
        public Translation2d location;
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
        OFF(0.0), LEFT_EJECT(-1.0), RIGHT_EJECT(1.0), VELOCITY(0.0);//Power Shifted to the Elevator
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
    public ControlState getBestSplitterState() {
        return bestSplitterState;
    }
    public void setOpenLoop(double demand) {
        splitter.set(ControlMode.PercentOutput, demand);
    }
    public void setVelocity(double rpm) {
        setState(ControlState.VELOCITY);
        splitter.set(ControlMode.Velocity, getRPMToEncVelocity(rpm));
    }

    public void conformToState(ControlState desiredState) {
        conformToState(desiredState, desiredState.speed);
    }
    public void conformToState(ControlState desiredState, double outputOverride) {
        setState(desiredState);
        setOpenLoop(outputOverride);
    }

    public double percentOutputToRPM(double percentOutput) {
        return 6380.0 * percentOutput;
    }
    public double getRPMToEncVelocity(double rpm) {
        return rpm * 2048.0 / 600.0 * 1.0;
    }
    public double calculateSurfaceVelocity() {
        Translation2d robotPosition = swerve.getPose().getTranslation();
        Rotation2d robotRotation = swerve.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(-90)).inverse();
        double distanceMagnitude = Math.cos(robotRotation.getRadians()) * robotPosition.x();
        

        return 0.0;
    }
    public double surfaceVelocityToRPM() {

        return 0.0;
    }

    public synchronized double encVelocityToRPM(double encVelocity) {
        return encVelocity / 2048.0 * 600.0 / 1.0;
    }
    
    

    public void updateBestEjectLocation() {
        boolean locationFound = false;
        double lowestLocationsMagnitude = Double.POSITIVE_INFINITY;
        EjectLocations closestLocation = EjectLocations.TEAM_HANGER;
        for(EjectLocations ejectLocation : ejectLocationsArray) {
            double locationMagnitude = ejectLocation.location.translateBy(swerve.pose.getTranslation().inverse()).norm();
            if(locationMagnitude < Constants.BallSplitter.kD2F && !locationFound) {
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
                                        rotateBy(Rotation2d.fromDegrees(90)).inverse());
        Rotation2d rightSideEjectDelta = robotToLocationTheta.rotateBy(robotRotation.inverse().
                                        rotateBy(Rotation2d.fromDegrees(-90)).inverse());
        if(Math.abs(rightSideEjectDelta.getDegrees()) < Math.abs(leftSideEjectDelta.getDegrees())) { //The right side is closer
            robotToLocationTheta.rotateBy(Rotation2d.fromDegrees(90));
        } else if(Math.abs(rightSideEjectDelta.getDegrees()) > Math.abs(leftSideEjectDelta.getDegrees())) {//The left side is closer
            robotToLocationTheta.rotateBy(Rotation2d.fromDegrees(-90));
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

    public double getEjectRotation(Translation2d target) {
        Translation2d targetPosition = target;
        Translation2d robotPosition = swerve.getPose().getTranslation();
        Rotation2d robotRotation = swerve.getPose().getRotation();
        Translation2d robotToTargetVector = targetPosition.translateBy(robotPosition.inverse());
        Rotation2d robotToTargetRotation = robotToTargetVector.direction();
        Rotation2d leftEjectRotation = robotToTargetRotation.rotateBy(Rotation2d.fromDegrees(-90));
        Rotation2d rightEjectRotation = robotToTargetRotation.rotateBy(Rotation2d.fromDegrees(90));
        if(Math.abs(robotRotation.distance(leftEjectRotation)) > Math.abs(robotRotation.distance(rightEjectRotation))) {
            return rightEjectRotation.getDegrees();
        } else {
            return leftEjectRotation.getDegrees();
        }
        //return robotToTerminalRotation.getDegrees();
    }

    Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            bestSplitterState = ControlState.RIGHT_EJECT;
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
        if(Settings.debugFeeder()) {
            SmartDashboard.putNumber("Swerve Rotation to BEL", targetSwerveTheta);
            SmartDashboard.putNumber("Swerve Raw Theta",  Util.boundAngle0to360Degrees(translationVector.direction().getDegrees()));
            SmartDashboard.putString("Eject Mode", bestSplitterState.toString());
        }
        /*if(splitter.getBusVoltage() == 0)
            DriverStation.reportError("SPLITTER MOTOR NOT DETECTED", false);*/
        
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
