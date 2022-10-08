// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class InductiveEncoder {
    DutyCycle encoder;
    private double lowBound = 0.104193;
    private double upperBound = 0.904299;

    double highestUpperBound = 0;
    double lowestUpperBound = 1;

    double offset = 0;
    public InductiveEncoder(int port) {
        encoder = new DutyCycle(new DigitalInput(port));
    }
    public double getOutput() {
        return encoder.getOutput();
    }
    public double getBoundedValue() {
        double output = getOutput();
        double targetLowBoundRange = 0;
        double targetHighBoundRange = 1;
        output = 0 + ((targetHighBoundRange - targetLowBoundRange) / (upperBound - lowBound)) * (output - lowBound);
        //System.out.println("Range = " + output + ", output = " + (upperBound - lowBound));
        return output;
    }
    public double getAbsolutePosition() {
        return getBoundedValue() * 360.0 - offset;
    }
    public double getFrequency() {
        return encoder.getFrequency();
    }
}
