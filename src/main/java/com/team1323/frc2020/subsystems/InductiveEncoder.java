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
    DutyCycleEncoder encoder;
    private double lowBound = 0.1015;
    private double upperBound = 0.907;

    double highestUpperBound = 0;
    double lowestUpperBound = 1;
    public double getHighestBound() {
        return highestUpperBound;
    }
    public double getLowestBound() {
        return lowestUpperBound;
    }

    double offset = 0;
    public InductiveEncoder(int port) {
        encoder = new DutyCycleEncoder(new DigitalInput(port));
        encoder.setDutyCycleRange(lowBound, upperBound);
    }
    public double getOutput() {
        return encoder.getAbsolutePosition();
    }
    public double getBoundedValue() {
        return getOutput();
    }
    public double getAbsolutePosition() {
        return getBoundedValue() * 360.0 - offset;
    }

    public double getRawPosition() {
        return getBoundedValue();
    }
    public double getFrequency() {
        return encoder.getFrequency();
    }
}
