package com.team1323.frc2020.subsystems.shooting;

import com.team1323.frc2020.Constants.MotorizedHood;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class CompBotRegularBallProfile extends ShootingProfile {

    private static final double kCloseRPMOffset = 0.0;
    private static final double kMidRPMOffset = 0.0;
    private static final double kFarRPMOffset = 0.0;

    private static final double kCloseHoodOffset = 0.0;
    private static final double kMidHoodOffset = 0.0;
    private static final double kFarHoodOffset = 0.0;
    //4 PSI
    private static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> kDistanceToShotVectorMap = new InterpolatingTreeMap<>();
    static {
        // Key: distance from the vision target, in inches
        // Value: a Translation2d whose direction represents a hood angle, and whose magnitude represents a shooter RPM
        kDistanceToShotVectorMap.put(new InterpolatingDouble(86.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 13.0 + kCloseHoodOffset), 1700.0 + kCloseRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(98.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 15.5 + kCloseHoodOffset), 1800.0 + kCloseRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(110.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 17.5 + kCloseHoodOffset), 1825.0 + kCloseRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(122.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 19.5 + kCloseHoodOffset), 1950.0 + kCloseRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(134.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 21.0 + kMidHoodOffset), 2025.0 + kMidRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(146.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 22.0 + kMidHoodOffset), 2100.0 + kMidRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(158.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 24.0 + kMidHoodOffset), 2200.0 + kMidRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(170.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 25.5 + kMidHoodOffset), 2450.0 + kMidRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(182.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0 + kFarHoodOffset), 2800.0 + kFarRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(194.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 29.0 + kFarHoodOffset), 2850.0 + kFarRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(206.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 29.0 + kFarHoodOffset), 2950.0 + kFarRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(218.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 30.0 + kFarHoodOffset), 3000.0 + kFarRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(228.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 30.0 + kFarHoodOffset), 3200.0 + kFarRPMOffset));
    }
    /*
    private static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> kDistanceToShotVectorMap = new InterpolatingTreeMap<>();
    static {
        // Key: distance from the vision target, in inches
        // Value: a Translation2d whose direction represents a hood angle, and whose magnitude represents a shooter RPM
        kDistanceToShotVectorMap.put(new InterpolatingDouble(86.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 12.0), 1650.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(98.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 14.0), 1750.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(110.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 15.0), 1850.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(122.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 19.0), 1900.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(134.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 19.0), 2050.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(146.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 23.0), 1850.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(158.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 24.0), 1850.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(170.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 26.0), 2025.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(182.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 2125.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(194.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2250.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(206.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2525.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(218.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 3025.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(230.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 3125.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(242.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 3160.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(256.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 3125.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(268.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3325.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(290.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3325.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(302.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3325.0));
    }*/

    private static final double kBallVelocityScrubFactor = 314.193746198 / 345.7617885;

    private static final double kRobotVelocityScalar = 0.9;

    private static final double kCloseTurretAngleInterpolation = 0.55;

    private static final double kFarTurretAngleInterpolation = 0.75;

    private static final double kRpmCompensationScalar = 1.75;

    private static final double kMaxEmpiricalHoodAngle = 70.3740995752 + 20.0;

    @Override
    public InterpolatingTreeMap<InterpolatingDouble, Translation2d> getDistanceToShotVectorMap() {
        return kDistanceToShotVectorMap;
    }

    @Override
    public double getBallVelocityScrubFactor() {
        return kBallVelocityScrubFactor;
    }

    @Override
    public double getRobotVelocityScalar() {
        return kRobotVelocityScalar;
    }

    @Override
    public double getCloseTurretAngleInterpolation() {
        return kCloseTurretAngleInterpolation;
    }

    @Override
    public double getFarTurretAngleInterpolation() {
        return kFarTurretAngleInterpolation;
    }

    @Override
    public double getRpmCompensationScalar() {
        return kRpmCompensationScalar;
    }

    @Override
    public double getMaxEmpiricalHoodAngle() {
        return kMaxEmpiricalHoodAngle;
    }
    
}
