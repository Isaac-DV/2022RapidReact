// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.auto.actions;

import com.team1323.frc2020.subsystems.BallFeeder;
import com.team1323.frc2020.subsystems.Column;
import com.team1323.frc2020.subsystems.BallFeeder.BallType;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class WaitForOneBallAction implements Action {
    Column column;
    BallFeeder ballFeeder;
    double timeout = 0;
    double startTimestamp = 0;

    public WaitForOneBallAction(double timeout) {
        column = Column.getInstance();
        ballFeeder = BallFeeder.getInstance();
        this.timeout = timeout;
        this.startTimestamp = Timer.getFPGATimestamp();
    }
    @Override
    public boolean isFinished() {
        return (ballFeeder.getDetectedBallType() == BallType.Team) || ((Timer.getFPGATimestamp() - startTimestamp) >= timeout);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

}
