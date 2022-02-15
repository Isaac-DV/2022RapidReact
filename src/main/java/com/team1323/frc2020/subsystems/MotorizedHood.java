// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.team1323.frc2020.Ports;

import edu.wpi.first.wpilibj.Servo;

/** Add your docs here. */
public class MotorizedHood extends Subsystem {
    Servo servo;
    
    private static MotorizedHood instance = null;
    
    public static MotorizedHood getInstance() {
        if(instance == null)
            instance = new MotorizedHood();
        return instance;
    }

    public MotorizedHood() {
        servo = new Servo(Ports.MOTORIZED_HOOD);
    }

    @Override
    public void outputTelemetry() {
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

}
