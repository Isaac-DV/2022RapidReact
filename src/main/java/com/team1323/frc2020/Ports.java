package com.team1323.frc2020;

public class Ports {
    //TODO: Update Ports with IRL Number
    
    // CAN Devices 
    // Falcons
    public static final int FRONT_RIGHT_ROTATION= 4; 
    public static final int FRONT_RIGHT_DRIVE   = 0; 
    public static final int FRONT_LEFT_ROTATION = 11;
    public static final int FRONT_LEFT_DRIVE    = 15; 
    public static final int REAR_LEFT_ROTATION  = 8;
    public static final int REAR_LEFT_DRIVE     = 12;
    public static final int REAR_RIGHT_ROTATION = 5;
    public static final int REAR_RIGHT_DRIVE    = 3;
    
    public static final int INTAKE = 14;

    public static final int TURRET = 6;
    
    public static final int SHOOTER_TOP = 7;
    public static final int SHOOTER_BOTTOM = 13;

    public static final int BALL_FEEDER = 2;
    public static final int BALL_EJECTOR = 0;
    public static final int BALL_SPLITTER = 0;

    public static final int UPPER_HOPPER = 0;

    public static final int HOOD_TALON = 10;


    public static final int HANGER = 1;
        
    // MISC CAN
    public static final int PIGEON = 22;
    public static final int SECONDARY_PIGEON = 23;
    
    //PWM
    public static final int HOOD_SERVO = 0;

    //Digital Inputs
    public static final int FRONT_RIGHT_ENCODER = 0;
    public static final int FRONT_LEFT_ENCODER = 1;
    public static final int REAR_LEFT_ENCODER = 2;
    public static final int REAR_RIGHT_ENCODER = 3;
    public static final int[] kModuleEncoders = new int[]{FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
        REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
    
    public static final int TURRET_ENCODER = 4;
    public static final int FEEDER_BANNER = 5;
    public static final int COLOR_SENSOR = 0;
    public static final int HOOD_ENCODER = 6;
    public static final int HANGER_ENCODER = 7;

    // Pneumatics
    public static final int PCM = 21;
    public static final int HOOD_FOLDER = 4;
    public static final int HOOD_EXTENDER = 5;
    public static final int INTAKE_EXTENDER = 1;
    public static final int WOF_EXTENDER = 3;
    public static final int HANGER_EXTENDER = 0;
    public static final int FEEDER_SHIFTER = 0;


    //Canifier
    public static final int CANIFIER = 30;
        
    }
    