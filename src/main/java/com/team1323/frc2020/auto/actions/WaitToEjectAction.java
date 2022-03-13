// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.auto.actions;

import com.team1323.frc2020.subsystems.BallFeeder;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class WaitToEjectAction implements Action {
    BallFeeder ballFeeder;


    double startTime;
    double timeout;
    public WaitToEjectAction(double timeout) {
        ballFeeder = BallFeeder.getInstance();

        this.timeout = timeout;
        this.startTime = Timer.getFPGATimestamp();
    }
    @Override
    public boolean isFinished() {
        System.out.println("detected ball type is = " + ballFeeder.getDetectedBallType().toString() + ": the time is = " + (Timer.getFPGATimestamp() - this.startTime));
        return (ballFeeder.getDetectedBallType() == BallFeeder.BallType.Opponent) || ((Timer.getFPGATimestamp() - this.startTime) >= this.timeout);
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

}
