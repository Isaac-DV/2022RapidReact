// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1323.frc2020.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2020.Constants;
import com.team1323.frc2020.auto.AutoModeBase;
import com.team1323.frc2020.auto.AutoModeEndedException;
import com.team1323.frc2020.auto.actions.ResetPoseAction;
import com.team1323.frc2020.auto.actions.SetTrajectoryAction;
import com.team1323.frc2020.auto.actions.WaitAction;
import com.team1323.frc2020.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2020.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SixBallOneEjectMode extends AutoModeBase {

    Superstructure s;
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
        return Arrays.asList(trajectories.firstBallBackup, trajectories.firstBallToHumanPlayer, 
                trajectories.humanPlayerToSecondOpponentBall, 
                trajectories.secondOpponentBallToThirdBall);
    }
    public SixBallOneEjectMode() {
        s = Superstructure.getInstance();
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.autoStartingPose));
        s.intakeState();
        runAction(new SetTrajectoryAction(trajectories.firstBallBackup, 90, 1));
        runAction(new WaitToFinishPathAction(5));
        runAction(new SetTrajectoryAction(trajectories.firstBallToHumanPlayer, 180, 1));
        runAction(new WaitToPassXCoordinateAction(150.0));
        s.swerve.setRotationScalar(0.5);
        s.swerve.setAbsolutePathHeading(135.0);
        runAction(new WaitToFinishPathAction(5));
        runAction(new WaitAction(2));
        s.postIntakeState();
        runAction(new SetTrajectoryAction(trajectories.humanPlayerToSecondOpponentBall, 270, 1));
        runAction(new WaitToFinishPathAction(5));
        s.intakeState();
        runAction(new SetTrajectoryAction(trajectories.secondOpponentBallToThirdBall, 270, 1));
        runAction(new WaitToFinishPathAction(5));
        s.postIntakeState();
        
    }

}
