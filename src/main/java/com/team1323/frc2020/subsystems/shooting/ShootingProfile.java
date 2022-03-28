package com.team1323.frc2020.subsystems.shooting;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Translation2d;

public abstract class ShootingProfile {
    
    public abstract InterpolatingTreeMap<InterpolatingDouble, Translation2d> getDistanceToShotVectorMap();

    public abstract double getBallVelocityScrubFactor();

    public abstract double getRobotVelocityScalar();

    public abstract double getTurretAngleInterpolation();

    public abstract double getRpmCompensationScalar();
}
