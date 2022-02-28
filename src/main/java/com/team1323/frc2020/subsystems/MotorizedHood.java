// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import java.util.Optional;

import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.loops.ILooper;
import com.team1323.frc2020.loops.Loop;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.frc2020.vision.ShooterAimingParameters;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class MotorizedHood extends Subsystem {
    Servo rightServo;
    Servo leftServo;
    double servoTargetAngle;
    double targetPosition = 0.0;
    boolean travelingUp = false;
    double finishTimestamp = 0.0; // timestamp when the hood is predicted to reach its target position
    private static MotorizedHood instance = null;
    
    public static MotorizedHood getInstance() {
        if(instance == null)
            instance = new MotorizedHood();
        return instance;
    }

    public enum State {
        POSITION, VISION
    }
    private State currentState = State.POSITION;
    public State getState() {
        return currentState;
    }
    public void setState(State newState) {
        currentState = newState;
    }

    public MotorizedHood() {
        rightServo = new Servo(Ports.HOOD_RIGHT_SERVO);
        leftServo = new Servo(Ports.HOOD_LEFT_SERVO);

        rightServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        leftServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        setAngle(Constants.MotorizedHood.kMinControlAngle);
    }
    
    private void setServoAngle(double angle) {
        angle = Util.limit(angle, Constants.MotorizedHood.kMinControlAngle, Constants.MotorizedHood.kMaxControlAngle);
        servoTargetAngle = angle;

        double angleRange = Constants.MotorizedHood.kMaxControlAngle - Constants.MotorizedHood.kMinControlAngle;
        double offsetAngle = angle - Constants.MotorizedHood.kMinControlAngle;
        double adjacentSideLength = 1.0 / Math.tan(Math.toRadians(angleRange));
        double oppositeSideLength = Math.tan(Math.toRadians(offsetAngle)) * adjacentSideLength;
        setServoPercentage(oppositeSideLength);
    }

    private void setServoPercentage(double percent) {
        rightServo.set(percent);
        leftServo.set(percent);

        double currentPosition = getPosition();
        double travelDistance = percent - currentPosition;
        travelingUp = travelDistance >= 0.0;
        finishTimestamp = Timer.getFPGATimestamp() + (Math.abs(travelDistance) / Constants.MotorizedHood.kServoSpeed);
        targetPosition = percent;
    }

    public void setAngle(double angle) {
        setState(State.POSITION);
        setServoAngle(angle);
    }

    public double getPosition() {
        double timestamp = Timer.getFPGATimestamp();
        if (timestamp >= finishTimestamp) {
            return targetPosition;
        }

        double remainingDistance = (finishTimestamp - timestamp) * Constants.MotorizedHood.kServoSpeed;
        if (!travelingUp)
            remainingDistance *= -1.0;

        return targetPosition - remainingDistance;
    }

    public double getAngle() {
        double angleRange = Constants.MotorizedHood.kMaxControlAngle - Constants.MotorizedHood.kMinControlAngle;
        double oppositeSideLength = getPosition();
        double adjacentSideLength = 1.0 / Math.tan(Math.toRadians(angleRange));

        return Constants.MotorizedHood.kMinControlAngle + Math.toDegrees(Math.atan(oppositeSideLength / adjacentSideLength));
    }

    public double getRightServoTargetPosition() {
        return rightServo.getPosition();
    }

    public double getLeftServoTargetPosition() {
        return leftServo.getPosition();
    }

    public static double physicalAngleToEmpiricalAngle(double physicalAngle) {
        double physicalAngleOffset = physicalAngle - Constants.MotorizedHood.kMinControlAngle;

        return Constants.MotorizedHood.kMaxEmpiricalAngle - physicalAngleOffset;
    }

    public static double empiricalAngleToPhysicalAngle(double empiricalAngle) {
        double empiricalAngleOffset = Constants.MotorizedHood.kMaxEmpiricalAngle - empiricalAngle;

        return Constants.MotorizedHood.kMinControlAngle + empiricalAngleOffset;
    }
    public boolean hasReachedAngle() {
        return Math.abs(servoTargetAngle - getAngle()) <= Constants.MotorizedHood.kAngleTolerance;
    }

    private Loop loop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            
        }

        @Override
        public void onLoop(double timestamp) {
            switch (currentState) {
                case VISION:
                    Optional<ShooterAimingParameters> aim = RobotState.getInstance().getAimingParameters();
                    if (aim.isPresent()) {
                        setServoAngle(aim.get().getHoodAngle().getDegrees());
                    }
                    break;
                default:
                    break;
            }
        }

        @Override
        public void onStop(double timestamp) {
            
        }

    };

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(loop);
    }

    public Request setAngleRequest(double desiredAngle) {
        return new Request() {

            @Override
            public void act() {
                setAngle(desiredAngle);
            }
            @Override
            public boolean isFinished() {
                return Math.abs(servoTargetAngle - getAngle()) <= Constants.MotorizedHood.kAngleTolerance;
            }
        };
    }

    public Request visionRequest() {
        return new Request() {

            @Override
            public void act() {
                setState(State.VISION);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(servoTargetAngle - getAngle()) <= Constants.MotorizedHood.kAngleTolerance;
            }
        };
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Servo Right Position", getRightServoTargetPosition());
        SmartDashboard.putNumber("Servo Left Position", getLeftServoTargetPosition());
        SmartDashboard.putNumber("Servo Target Angle", servoTargetAngle);
    }

    @Override
    public void stop() {
        
    }

}
