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
import com.team1323.frc2020.auto.actions.WaitForShotsAction;
import com.team1323.frc2020.auto.actions.WaitForSuperstructureAction;
import com.team1323.frc2020.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2020.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2020.subsystems.Intake;
import com.team1323.frc2020.subsystems.MotorizedHood;
import com.team1323.frc2020.subsystems.Shooter;
import com.team1323.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TwoBallTwoEjectMode extends AutoModeBase {
    Superstructure s;
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
        return Arrays.asList(trajectories.thirdBallBackup, trajectories.thirdBallToSecondOpponentBall,
            trajectories.secondOpponentBallToThirdOpponentBall
        );
    }
    public TwoBallTwoEjectMode() {
        s = Superstructure.getInstance();
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.autoRightStartingPose));
        s.intake.conformToState(Intake.ControlState.EJECT);
        s.turret.startVision();
        s.motorizedHood.setState(MotorizedHood.State.VISION);
        s.shooter.setState(Shooter.State.VISION); 
        runAction(new SetTrajectoryAction(trajectories.thirdBallBackup, -135.0, 1.0));
        runAction(new WaitAction(0.5));
        s.intakeState();
        runAction(new WaitToFinishPathAction(4));
        s.visionShotState();
        runAction(new WaitForSuperstructureAction());
        runAction(new WaitForShotsAction(2.0));
        s.postShotState();
        runAction(new WaitForSuperstructureAction());
        runAction(new SetTrajectoryAction(trajectories.thirdBallToSecondOpponentBall, 135.0, 1.0));
        s.turret.startVision();
        s.intakeState();
        runAction(new WaitForSuperstructureAction());
        runAction(new WaitToFinishPathAction(7));
        runAction(new WaitAction(2));
        s.wrist.setWristAngle(Constants.Wrist.kStowedAngle);
        s.intake.conformToState(Intake.ControlState.OFF);
        runAction(new SetTrajectoryAction(trajectories.secondOpponentBallToThirdOpponentBall, -90, 1));
        runAction(new WaitToPassXCoordinateAction(233.0));
        s.intakeState();
        runAction(new WaitToFinishPathAction(10.0));
        runAction(new WaitAction(1.0));
        s.swerve.rotate(0);
        System.out.println("Auto mode finished in " + currentTime() + " seconds");

    }

}
