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
import com.team1323.frc2020.auto.actions.WaitToEjectAction;
import com.team1323.frc2020.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2020.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2020.auto.actions.WaitToPassYCoordinateAction;
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
        // Startup
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.autoLeftStartingPose));
        s.turret.setCOFState();
        s.motorizedHood.setState(MotorizedHood.State.VISION);
        s.column.setState(Column.ControlState.OFF);
        s.shooter.setState(Shooter.State.VISION);

        // Pick up first ball
        runAction(new SetTrajectoryAction(trajectories.thirdBallBackup, -135.0, 1.0));
        runAction(new WaitAction(0.5));
        s.intakeState();
        runAction(new WaitToFinishPathAction(4));

        // Shoot first balls
        s.visionShotState();
        runAction(new WaitForSuperstructureAction());
        runAction(new WaitForShotsAction(2.0));

        // Pick up second ball and eject it
        runAction(new SetTrajectoryAction(trajectories.thirdBallToSecondOpponentBall, -261, 0.5));
        s.turret.startVision();
        s.intakeState();
        runAction(new WaitAction(0.5));
        s.shooter.stop();
        runAction(new WaitToEjectAction(5));
        runAction(new WaitAction(0.5));
        s.wrist.setWristAngle(Constants.Wrist.kStowedAngle);
        s.intake.conformToState(Intake.ControlState.OFF);

        // Pick up third ball and eject it
        runAction(new SetTrajectoryAction(trajectories.secondOpponentBallToThirdOpponentBall, -98, 0.5));
        runAction(new WaitToPassYCoordinateAction(-70));
        s.intakeState();
        runAction(new WaitToFinishPathAction(10.0));
        runAction(new WaitAction(1.0));

        // Rotate to face forward
        s.swerve.rotate(0);
        System.out.println("Auto mode finished in " + currentTime() + " seconds");

    }

}
