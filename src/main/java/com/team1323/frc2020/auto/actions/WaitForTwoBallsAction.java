// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.auto.actions;

import com.team1323.frc2020.subsystems.BallFeeder;
import com.team1323.frc2020.subsystems.Column;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class WaitForTwoBallsAction implements Action {
    Column column;
    BallFeeder ballFeeder;
    double timeout = 0;
    double startTimestamp = 0;

    public WaitForTwoBallsAction(double timeout) {
        column = Column.getInstance();
        ballFeeder = BallFeeder.getInstance();
        this.timeout = timeout;
        this.startTimestamp = Timer.getFPGATimestamp();
    }
    @Override
    public boolean isFinished() {
        return column.getBanner() && ballFeeder.isTeamBallDetected() || ((Timer.getFPGATimestamp() - startTimestamp) >= timeout);
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
