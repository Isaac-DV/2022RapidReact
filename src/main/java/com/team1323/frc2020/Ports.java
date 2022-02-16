package com.team1323.frc2020;

public class Ports {
    //TODO: Update Ports with IRL Number
    
    // CAN Devices 
    // Falcons
    public static final int FRONT_RIGHT_ROTATION= 1; 
    public static final int FRONT_RIGHT_DRIVE   = 0;

    public static final int FRONT_LEFT_ROTATION = 17;
    public static final int FRONT_LEFT_DRIVE    = 18; 

    public static final int REAR_LEFT_ROTATION  = 11;
    public static final int REAR_LEFT_DRIVE     = 10;

    public static final int REAR_RIGHT_ROTATION = 8;
    public static final int REAR_RIGHT_DRIVE    = 9;
    

    public static final int INTAKE = 2;
    public static final int WRIST = 3;
    public static final int BALL_SPLITTER = 12;
    public static final int BALL_EJECTOR = 16;
    public static final int BALL_FEEDER = 13;
    public static final int TURRET = 6;    
    public static final int SHOOTER_TOP = 7;
    public static final int SHOOTER_BOTTOM = 0;


    public static final int HANGER_WRIST = 1;

    // MISC CAN
    public static final int PIGEON = 45;
    
    //PWM
    public static final int MOTORIZED_HOOD = 0;

    //Digital Inputs
    public static final int FRONT_RIGHT_ENCODER = 30;
    public static final int FRONT_LEFT_ENCODER = 31;
    public static final int REAR_LEFT_ENCODER = 32;
    public static final int REAR_RIGHT_ENCODER = 33;
    public static final int[] kModuleEncoders = new int[]{FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
        REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
    //CANCoders
    public static final int TURRET_ENCODER = 4;
    public static final int FEEDER_BANNER = 5;
    public static final int COLOR_SENSOR = 0;
    public static final int WRIST_ENCODER = 0;
    public static final int HANGER_WRIST_ENCODER = 0;

    // Pneumatics
    public static final int PCM = 21;
    public static final int FEEDER_SHIFTER = 0;
    public static final int ELEVATOR_SHIFTER = 0;
    public static final int TELESCOPE = 0;
    public static final int CLAW = 0;

    //Canifier
    public static final int CANIFIER = 30;
        
    }
    