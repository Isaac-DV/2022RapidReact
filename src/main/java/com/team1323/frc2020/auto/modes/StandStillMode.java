package com.team1323.frc2020.auto.modes;

import com.team1323.frc2020.auto.AutoModeBase;
import com.team1323.frc2020.auto.AutoModeEndedException;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot standstill
 */
public class StandStillMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting Stand Still Mode... Done!");
    }
}