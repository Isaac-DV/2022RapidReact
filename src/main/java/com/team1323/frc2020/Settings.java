/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2020;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * 
 */
public class Settings {

    private static Settings instance = new Settings(); 

    public static final boolean kIsUsingCompBot = false;
	public static final boolean kIsUsingTractionWheels = true;

    public static final boolean kSimulate = false;
	public static final boolean kResetTalons = false;
    
    // Separate debugging output into the different subsystems so as to not 
    // overload the NetworkTables
    private boolean kDebugSwerve = false;
    private boolean kDebugTurret = false;
    private boolean kDebugShooter = false;
    private boolean kDebugVision = false;
    private boolean kDebugFeeder = false;
    private boolean kDebugHanger = false;
    private boolean kDebugHood = false;
    private boolean kDebugColumn = false;

    private NetworkTableEntry swerveToggle;
    private NetworkTableEntry turretToggle;
    private NetworkTableEntry shooterToggle;
    private NetworkTableEntry visionToggle;
    private NetworkTableEntry feederToggle;
    private NetworkTableEntry hangerToggle;
    private NetworkTableEntry hoodToggle;
    private NetworkTableEntry columnToggle;

    private final String TAB = "Settings";

    private void putToggles() {
        swerveToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Swerve", kDebugSwerve).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        turretToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Turret", kDebugTurret).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        shooterToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Shooter", kDebugShooter).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        visionToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Vision", kDebugVision).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        feederToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Feeder", kDebugFeeder).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        hangerToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Hanger", kDebugHanger).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        hoodToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Hood", kDebugHood).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
        columnToggle = Shuffleboard.getTab(TAB).addPersistent("Debug Column", kDebugColumn).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    }

    private void updateSettings() {
        instance.kDebugSwerve = swerveToggle.getBoolean(instance.kDebugSwerve);
        instance.kDebugTurret = turretToggle.getBoolean(instance.kDebugTurret);
        instance.kDebugShooter = shooterToggle.getBoolean(instance.kDebugShooter);
        instance.kDebugVision = visionToggle.getBoolean(instance.kDebugVision);
        instance.kDebugFeeder = feederToggle.getBoolean(instance.kDebugFeeder);
        instance.kDebugHanger = hangerToggle.getBoolean(instance.kDebugHanger);
        instance.kDebugHood = hoodToggle.getBoolean(instance.kDebugHood);
        instance.kDebugColumn = columnToggle.getBoolean(instance.kDebugColumn);
    }

    public static void initializeToggles() {
        instance.putToggles();
    }

    public static void update() {
        instance.updateSettings();
    }

    public static boolean debugSwerve(){ return instance.kDebugSwerve; }
    public static boolean debugTurret(){ return instance.kDebugTurret; }
    public static boolean debugShooter(){ return instance.kDebugShooter; }
    public static boolean debugVision(){ return instance.kDebugVision; }
    public static boolean debugFeeder(){ return instance.kDebugFeeder ;}
    public static boolean debugHanger(){ return instance.kDebugHanger ;}
    public static boolean debugHood(){ return instance.kDebugHood ;}
    public static boolean debugColumn(){ return instance.kDebugColumn ;}

}
