// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.auto.actions;

import com.team1323.frc2020.subsystems.Column;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class WaitForShotsAction implements Action {
    Column column;

    double timeout;
    double startTimestamp;
    int ballsFired = 0;
    int targetBallCount = 0;
    boolean previousIsShootingBall = false;

    public WaitForShotsAction(double timeout) {
        this(timeout, 2);
    }
    public WaitForShotsAction(double timeout, int targetBallCount) {
        column = Column.getInstance();
        this.timeout = timeout;
        this.startTimestamp = Timer.getFPGATimestamp();
        this.targetBallCount = targetBallCount;
    }


    @Override
    public boolean isFinished() {
        if (ballsFired >= targetBallCount && !column.isShootingCurrentBall()) {
            System.out.println("WaitForShotsAction finished because we shot " + ballsFired + " balls");
            return true;
        }
        if ((Timer.getFPGATimestamp() - startTimestamp) >= timeout) {
            System.out.println("WaitForShotsAction timed out");
            return true;
        }

        return false;
    }

    @Override
    public void start() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void update() {
        boolean isShooting = column.isShootingCurrentBall();
        if (!previousIsShootingBall && isShooting) {
            ballsFired++;
        }
        previousIsShootingBall = isShooting;
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

}
