// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.led.CANdle;

/** Add your docs here. */
public class LEDs extends Subsystem {
    CANdle candle;

    private static LEDs instance = null;
    public static LEDs getInstance() {
        if(instance == null)
            instance = new LEDs();
        return instance;
    }

    public LEDs() {
        candle = new CANdle(0, "main");
    }
    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }
    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
}
