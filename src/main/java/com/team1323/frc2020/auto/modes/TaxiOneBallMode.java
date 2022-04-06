// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.auto.modes;

import com.team1323.frc2020.Constants;
import com.team1323.frc2020.auto.AutoModeBase;
import com.team1323.frc2020.auto.AutoModeEndedException;
import com.team1323.frc2020.auto.actions.ResetPoseAction;
import com.team1323.frc2020.auto.actions.SetTrajectoryAction;
import com.team1323.frc2020.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2020.subsystems.Column;
import com.team1323.frc2020.subsystems.Intake;
import com.team1323.frc2020.subsystems.MotorizedHood;
import com.team1323.frc2020.subsystems.Shooter;
import com.team1323.frc2020.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TaxiOneBallMode extends AutoModeBase { 
    Superstructure s;
    public TaxiOneBallMode() {
        s = Superstructure.getInstance();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.autoLeftFarStartingPose));

        s.intake.conformToState(Intake.ControlState.EJECT);
        s.turret.setCOFState();
        s.motorizedHood.setState(MotorizedHood.State.VISION);
        s.column.setState(Column.ControlState.OFF);
        s.shooter.setState(Shooter.State.VISION);
        
        //Pass the tarmac line
        runAction(new SetTrajectoryAction(trajectories.farLeftTaxiBackup, -135.0, 1.0));
        runAction(new WaitToFinishPathAction(5.0));
        s.visionShotState();
        System.out.println("Auto mode finished in " + currentTime() + " seconds");

    }

}
