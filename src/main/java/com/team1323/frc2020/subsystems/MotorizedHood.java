// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.frc2020.subsystems.requests.Request;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class MotorizedHood extends Subsystem {
    Servo rightServo;
    Servo leftServo;
    double servoTargetAngle;
    double targetPosition;
    private static MotorizedHood instance = null;
    
    public static MotorizedHood getInstance() {
        if(instance == null)
            instance = new MotorizedHood();
        return instance;
    }

    public MotorizedHood() {
        rightServo = new Servo(Ports.HOOD_RIGHT_SERVO);
        leftServo = new Servo(Ports.HOOD_LEFT_SERVO);

        rightServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        leftServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        setServoAngle(Constants.MotorizedHood.kMinControlAngle);
    }
    
    public void setServoAngle(double angle) {
        angle = Util.limit(angle, Constants.MotorizedHood.kMinControlAngle, Constants.MotorizedHood.kMaxControlAngle);
        servoTargetAngle = angle;

        double angleRange = Constants.MotorizedHood.kMaxControlAngle - Constants.MotorizedHood.kMinControlAngle;
        double offsetAngle = angle - Constants.MotorizedHood.kMinControlAngle;
        double adjacentSideLength = 1.0 / Math.tan(Math.toRadians(angleRange));
        double oppositeSideLength = Math.tan(Math.toRadians(offsetAngle)) * adjacentSideLength;
        targetPosition = oppositeSideLength;
        setServoPercentage(oppositeSideLength);
    }
    public void setServoPercentage(double percent) {
        rightServo.set(percent);
        leftServo.set(percent);
    }
    public double getRightServoPosition() {
        return rightServo.getPosition();
    }
    public double getLeftServoPosition() {
        return leftServo.getPosition();
    }

    public Request setAngleRequest(double desiredAngle) {
        return new Request() {

            double startTimestamp = 0.0;

            @Override
            public void act() {
                startTimestamp = Timer.getFPGATimestamp();
                setServoAngle(desiredAngle);
            }
            @Override
            public boolean isFinished() {
                return Timer.getFPGATimestamp() - startTimestamp >= 0.5;
            }
        };
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Servo Right Position", getRightServoPosition());
        SmartDashboard.putNumber("Servo Left Position", getLeftServoPosition());
        SmartDashboard.putNumber("Servo Target Angle", servoTargetAngle);
    }

    @Override
    public void stop() {
        
    }

}
