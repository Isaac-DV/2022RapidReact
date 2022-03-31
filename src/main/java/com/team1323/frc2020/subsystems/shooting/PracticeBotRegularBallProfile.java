package com.team1323.frc2020.subsystems.shooting;

import com.team1323.frc2020.Constants.MotorizedHood;
import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class PracticeBotRegularBallProfile extends ShootingProfile {

    private static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> kDistanceToShotVectorMap = new InterpolatingTreeMap<>();
    static {
        // Key: distance from the vision target, in inches
        // Value: a Translation2d whose direction represents a hood angle, and whose magnitude represents a shooter RPM
        kDistanceToShotVectorMap.put(new InterpolatingDouble(86.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 12.0), 1800.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(98.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 14.0), 1900.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(110.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 15.0), 2000.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(122.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 19.0), 2050.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(134.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 19.0), 2200.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(146.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 23.0), 2200.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(158.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 24.0), 2200.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(170.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 26.0), 2200.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(182.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 2300.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(194.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2400.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(206.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2550.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(218.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2950.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(230.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 3050.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(242.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 3085.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(256.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 3150.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(268.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3250.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(290.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3250.0));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(302.6875), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3250.0));
    }

    private static final double kBallVelocityScrubFactor = 290.570415188 / 426.73300175;

    private static final double kRobotVelocityScalar = 0.7;

    private static final double kCloseTurretAngleInterpolation = 0.8;

    private static final double kFarTurretAngleInterpolation = 0.875;

    private static final double kRpmCompensationScalar = 1.25;

    private static final double kMaxEmpiricalHoodAngle = 65.12195 + 23.0;

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
