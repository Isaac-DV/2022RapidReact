// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.team1323.frc2020.Ports;

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
        candle = new CANdle(Ports.CANDLE);
        candle.configLEDType(LEDStripType.RGB);
        CANdleConfiguration config = new CANdleConfiguration();
        candle.configAllSettings(config);
        //candle.setLEDs(255, 0, 0);
    }
    public enum LEDMode {
        SOLID, RAINBOW, FIRE, TWINKLE;
    }
    private LEDMode selectedLEDType = LEDMode.SOLID;
    public LEDMode getLEDType() {
        return selectedLEDType;
    }
    public enum LEDColors {
        OFF(0,0,0, LEDMode.SOLID), RED(255,0,0, LEDMode.SOLID), GREEN(0,255,0, LEDMode.SOLID), BLUE(0,0,255, LEDMode.SOLID),
        DISABLED(255,0,0, LEDMode.SOLID), ENABLED(0,0,255, LEDMode.SOLID),
        RAINBOW(0,0,0, LEDMode.RAINBOW), FIRE(0,0,0, LEDMode.FIRE), TWINKLE(0,0,0, LEDMode.TWINKLE);
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
    private LEDColors currentLEDMode = LEDColors.RED;
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
            RainbowAnimation animation = new RainbowAnimation(0.25, 0.25, 1690);
            candle.animate(animation);
        } else if(selectedLEDType == LEDMode.FIRE) {
            FireAnimation fireAnimation = new FireAnimation(1, 1, 1690, 1, 0.25);
            candle.animate(fireAnimation);
        } else if(selectedLEDType == LEDMode.TWINKLE) {
            TwinkleAnimation twinkleAnimation = new TwinkleAnimation(255, 255, 255, 127, 0.25, 1690, TwinklePercent.Percent76);
            candle.animate(twinkleAnimation);
        }
    }
    @Override
    public void outputTelemetry() {

    }
    
    @Override
    public void stop() {
        //configLEDs(LEDColors.GREEN);
    }
}
