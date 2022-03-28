package com.team1323.frc2020.loops;

import com.team1323.frc2020.RobotState;
import com.team1323.frc2020.subsystems.Swerve;
import com.team1323.frc2020.subsystems.Turret;
import com.team254.lib.geometry.Rotation2d;

public class RobotStateEstimator implements Loop{
	private static RobotStateEstimator instance = null;
	public static RobotStateEstimator getInstance(){
		if(instance == null)
			instance = new RobotStateEstimator();
		return instance;
	}
	
	RobotStateEstimator() {
	}
	
	RobotState robotState = RobotState.getInstance();
	Swerve swerve;
	Turret turret;

	@Override
	public void onStart(double timestamp) {
		swerve = Swerve.getInstance();
		turret = Turret.getInstance();
	}

	@Override
	public void onLoop(double timestamp) {
		robotState.addObservations(timestamp, swerve.getPose(), swerve.getVelocity(), Rotation2d.fromDegrees(turret.getAngle()));
		robotState.getAimingParameters();
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}