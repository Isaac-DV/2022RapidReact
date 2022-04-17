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
import com.team1323.frc2020.subsystems.BallFeeder;
import com.team1323.frc2020.subsystems.Column;
import com.team1323.frc2020.subsystems.Intake;
import com.team1323.frc2020.subsystems.MotorizedHood;
import com.team1323.frc2020.subsystems.Shooter;
import com.team1323.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TwoBallBackHubHideMode extends AutoModeBase {
    Superstructure s;
    @Override
    public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
        return Arrays.asList(trajectories.thirdBallBackup, trajectories.thirdBallToThirdOpponentBall,
                trajectories.opponentBallToBackEjectLocation, trajectories.backSideEjectToSecondOpponentBall,
                trajectories.secondOpponentToBackSideEject
                );
    }
    public TwoBallBackHubHideMode() {
        s = Superstructure.getInstance();
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.autoLeftStartingPose));

        s.intake.conformToState(Intake.ControlState.EJECT);
        s.turret.setCOFState();
        s.column.conformToState(Column.ControlState.OFF);
        s.motorizedHood.setState(MotorizedHood.State.VISION);
        s.shooter.setState(Shooter.State.VISION);
        
        // Pick up first ball
        s.intakeState();
        runAction(new WaitForSuperstructureAction());
        runAction(new SetTrajectoryAction(trajectories.thirdBallBackup, -135.0, 1.0));
        runAction(new WaitAction(0.5));
        runAction(new WaitToFinishPathAction(4));

        // Shoot first balls
        s.visionShotState();
        runAction(new WaitForSuperstructureAction());
        runAction(new WaitForShotsAction(2.0));

        //Go To the opponents ball and intake(do not eject)
        runAction(new SetTrajectoryAction(trajectories.thirdBallToThirdOpponentBall, -90.0, 0.8));
        s.intakeState();
        runAction(new WaitForSuperstructureAction());
        s.ballFeeder.setState(BallFeeder.State.HOLD);
        runAction(new WaitToFinishPathAction(4));
        runAction(new WaitAction(0.5));
        
        //Go To the back side of the hub and poop el ball
        if(s.ballFeeder.hasIntakeResuckBeenActivated()) {
            s.wrist.setWristAngle(80.0);
            runAction(new SetTrajectoryAction(trajectories.opponentBallToBackEjectLocation, 21.0, 0.7));
            runAction(new WaitToFinishPathAction(6.0));
            s.intake.setOpenLoop(-0.15);
            runAction(new WaitAction(1.0));

            //go to the next ball and intake it
            runAction(new SetTrajectoryAction(trajectories.backSideEjectToSecondOpponentBall, 135.0, 1.0));
            s.intakeState();
            runAction(new WaitForSuperstructureAction());
            s.ballFeeder.setState(BallFeeder.State.HOLD);
            runAction(new WaitToFinishPathAction(6.0));

            //take the ball to the back side
            runAction(new WaitAction(0.5));
            s.wrist.setWristAngle(80.0);
            runAction(new SetTrajectoryAction(trajectories.secondOpponentToBackSideEject, 21.0, 0.7));
            runAction(new WaitToFinishPathAction(6.0));
            s.intake.setOpenLoop(-0.15);



        }
        System.out.println("Auto mode finished in " + currentTime() + " seconds");
    }
}
