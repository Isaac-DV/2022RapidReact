package com.team254.lib.trajectory.timing;

import com.team1323.frc2020.Constants;
import com.team254.lib.geometry.Pose2dWithCurvature;

public class CurvatureVelocityConstraint implements TimingConstraint<Pose2dWithCurvature>{

	@Override
	public double getMaxVelocity(final Pose2dWithCurvature state){
		return Constants.kSwerveMaxSpeedInchesPerSecond / (1 + Math.abs(50.0*state.getCurvature()));//4.0
	}
	
	@Override
	public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithCurvature state, final double velocity){
		return MinMaxAcceleration.kNoLimits;
	}
	
}
