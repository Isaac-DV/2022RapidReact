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
    public static final int BALL_EJECTOR = 39;
    public static final int BALL_FEEDER = 13;
    public static final int COLUMN = 5;
    public static final int TURRET = 14;    
    //public static final int SHOOTER_TOP = 7;
    public static final int SHOOTER_LEFT = 20;
    public static final int SHOOTER_RIGHT = 7;
    public static final int ELEVATOR = 16;



    public static final int HANGER_WRIST = 20;

    // MISC CAN
    public static final int PIGEON = 45;
    
    //PWM
    public static final int HOOD_RIGHT_SERVO = 0;
    public static final int HOOD_LEFT_SERVO = 1;

    //Digital Inputs
    public static final int FRONT_RIGHT_ENCODER = 30;
    public static final int FRONT_LEFT_ENCODER = 31;
    public static final int REAR_LEFT_ENCODER = 32;
    public static final int REAR_RIGHT_ENCODER = 33;
    public static final int[] kModuleEncoders = new int[]{FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
        REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER};
    //CANCoders
    public static final int TURRET_ENCODER = 2;
    public static final int FEEDER_BANNER = 4;
    public static final int COLUMN_BANNER = 6;
    public static final int COLOR_SENSOR = 5;
    public static final int WRIST_ENCODER = 1;
    public static final int HANGER_WRIST_ENCODER = 0;
    public static final int ELEVATOR_ENCODER = 3;

    // Pneumatics
    public static final int PCM = 41;
    public static final int TELESCOPE = 2;
    public static final int CLAW = 3;

    //Canifier
    public static final int CANIFIER = 30;
        
    }
    