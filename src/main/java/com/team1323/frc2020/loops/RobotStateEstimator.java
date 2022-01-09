package com.team1323.frc2020.loops;

import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.subsystems.Swerve;
import com.team254.lib.geometry.Rotation2d;

public class RobotStateEstimator implements Loop{
	private static RobotStateEstimator instance = null;
	public static RobotStateEstimator getInstance(){
		if(instance == null)
			instance = new RobotStateEstimator();
		return instance;
	}
	
	RobotStateEstimator(){
	}
	
	RobotState robotState = RobotState.getInstance();
	Swerve swerve;

	@Override
	public void onStart(double timestamp) {
		swerve = Swerve.getInstance();
	}

	@Override
	public void onLoop(double timestamp) {
		robotState.addObservations(timestamp, swerve.getPose(), swerve.getVelocity(), Rotation2d.identity());
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}
