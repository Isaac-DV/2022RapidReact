// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LEDs extends Subsystem {
    CANdle candle;

    public int mRed = 0;
    public int mGreen = 0;
    public int mBlue = 0;
    
    private static LEDs instance = null;
    public static LEDs getInstance() {
        if(instance == null)
            instance = new LEDs();
        return instance;
    }

    public LEDs() {
        candle = new CANdle(0, "main");
        candle.configLEDType(LEDStripType.RGB);
    }
    public enum LEDMode {
        SOLID, RAINBOW;
    }
    private LEDMode selectedLEDType = LEDMode.SOLID;
    public LEDMode getLEDType() {
        return selectedLEDType;
    }
    public enum LEDColors {
        OFF(0,0,0, LEDMode.SOLID), RED(255,0,0, LEDMode.SOLID), GREEN(0,255,0, LEDMode.SOLID), BLUE(0,0,255, LEDMode.SOLID),
        DISABLED(255,30,20, LEDMode.SOLID), ENABLED(0,0,255, LEDMode.SOLID),
        RAINBOW(0,0,0, LEDMode.RAINBOW);
        int r;
        int g;
        int b;
        LEDMode ledMode;
        LEDColors(int r,int g,int b, LEDMode ledMode) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.ledMode = ledMode;
        }
    }
    private LEDColors currentLEDMode = LEDColors.OFF;
    public LEDColors getLEDMode() {
        return currentLEDMode;
    }
    public void configLEDs(LEDColors ledColors) {
        this.mRed = ledColors.r;
        this.mGreen = ledColors.g;
        this.mBlue = ledColors.b;
        this.selectedLEDType = ledColors.ledMode;
    }

    @Override
    public void writePeriodicOutputs() {
        if(selectedLEDType == LEDMode.SOLID) {
            candle.setLEDs(mRed, mGreen, mBlue);
        } else if(selectedLEDType == LEDMode.RAINBOW) {
            RainbowAnimation animation = new RainbowAnimation(0.75, 1, 1690);
            candle.animate(animation);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("LED Mode", selectedLEDType.toString());
        SmartDashboard.putString("LED Color", currentLEDMode.toString());
    }
    @Override
    public void stop() {
        candle.setLEDs(255, 20, 30);
    }
}
