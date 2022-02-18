// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.team1323.frc2020.Constants;
import com.team1323.frc2020.Ports;
import com.team1323.lib.util.Util;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class MotorizedHood extends Subsystem {
    Servo servo;
    double servoTargetAngle;
    private static MotorizedHood instance = null;
    
    public static MotorizedHood getInstance() {
        if(instance == null)
            instance = new MotorizedHood();
        return instance;
    }

    public MotorizedHood() {
        servo = new Servo(Ports.MOTORIZED_HOOD);
    }
    
    public void setServoAngle(double angle) {
        angle = Util.limit(angle, Constants.MotorizedHood.kMinControlAngle, Constants.MotorizedHood.kMaxControlAngle);
        servoTargetAngle = angle;
        servo.setAngle(angle);
    }
    public double getServoAngle() {
        return servo.getAngle();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Servo Angle", getServoAngle());
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

}
