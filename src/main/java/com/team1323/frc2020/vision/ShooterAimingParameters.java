package com.team1323.frc2020.vision;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/**
 * A container class to specify the shooter angle. It contains the desired range, the field_to_goal_angle
 */
public class ShooterAimingParameters {
    double range;
    double last_seen_timestamp;
    double stability;
    Rotation2d turret_angle;
    Translation2d turret_to_goal;
    Rotation2d targetOrientation;

    public ShooterAimingParameters(double range, Rotation2d turret_angle, Translation2d turret_to_goal, double last_seen_timestamp,
            double stability) {
        this.range = range;
        this.turret_angle = turret_angle;
        this.turret_to_goal = turret_to_goal;
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getTurretAngle() {
        return turret_angle;
    }

    public Translation2d getTurretToGoal() {
        return turret_to_goal;
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }

    public Rotation2d getTargetOrientation(){
        return targetOrientation;
    }

}
