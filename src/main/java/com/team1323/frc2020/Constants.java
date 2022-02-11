package com.team1323.frc2020;

import java.util.Arrays;
import java.util.List;

import com.team1323.lib.util.InterpolatingDouble;
import com.team1323.lib.util.InterpolatingTreeMap;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class Constants {
    /*All distance measurements are in inches, unless otherwise noted.*/
    
    public static final double kLooperDt = 0.02;
    public static final double kAutoAimPredictionTime = 0.14;

    public static final double kEpsilon = 0.0001;
    
    //Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 36.5; //35.5
    public static final double kRobotLength = 32.25; //35.5
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    
    public static final double kFieldLength = 629.25;
    
    //Field Landmarks
    public static final Pose2d kRobotStartingPose = new Pose2d(new Translation2d(kFieldLength - (120.0 + 2.0 + kRobotHalfLength), (161.625 - 94.66)), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kPartnerRobotStartingPose = new Pose2d(new Translation2d(kFieldLength - (120.0 + 2.0 + kRobotHalfLength), (161.625 - 27.75)), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kAltRobotStartingPose = new Pose2d(new Translation2d(kFieldLength - (120.0 + 2.0 + kRobotHalfLength), -(161.625 - 27.75)), Rotation2d.fromDegrees(180.0));
    
    public static final Pose2d kTopRightQuadrantPose = new Pose2d(new Translation2d(605.0, -118.0), new Rotation2d());
    public static final Pose2d kTopLeftQuadrantPose = new Pose2d(new Translation2d(5.0, -152.0), new Rotation2d());
    public static final Pose2d kBottomLeftQuadrantPose = new Pose2d(new Translation2d(44.0, 115.0), new Rotation2d());
    public static final Pose2d kBottomRightQuadrantPose = new Pose2d(new Translation2d(632.0, 150), new Rotation2d());
    public static final List<Pose2d> kFieldCornerPositions = Arrays.asList(kTopRightQuadrantPose, kTopLeftQuadrantPose,
        kBottomLeftQuadrantPose, kBottomRightQuadrantPose);
    /**
    * Target Specifications
    */
    public static final double kVisionTargetHeight = 98.25; //81.0 to bottom
    public static final Rotation2d kPortTargetOrientation = Rotation2d.fromDegrees(0.0);
    public static final Translation2d kOuterPortToInnerPort = new Translation2d(29.25, 0.0);
    
    //Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 21.0;
    public static final double kWheelbaseWidth = 21.0;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants (X and Y are with respect to the turret's center)
    public static final double kCameraYOffset = 0.0;//0.25
    public static final double kCameraXOffset = 8.216; //8.5
    public static final double kCameraZOffset = 25.0; //26.776 24.524
    public static final double kCameraYawAngleDegrees = -1.0;//-12.7
    public static final double kCameraPitchAngleDegrees = Settings.kIsUsingCompBot ? 0.0 : 35.5; //21.75 for bottom 34.3 37.0604

    //Limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    
    //Goal tracker constants
    public static final double kMaxGoalTrackAge = 0.5;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kCameraFrameRate = 90.0;
    public static final double kTrackStabilityWeight = 1.0;
    public static final double kTrackAgeWeight = 1.0;
    public static final double kTrackSwitchingWeight = 0.0;
    public static final double kClosestVisionDistance = 26.0;//36.0
    
    public static final double kVisionPIDOutputPercent = 0.5;

    public static final double kPosePredictionTime = 0.125; // seconds 0.25
    
    public static final double kDistanceToTargetTolerance = 1.0;

    public static final double kGyroDriftPerRotation = -0.25; // degrees
    
    //Path following constants
    public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
    public static final double kPathMinLookaheadDistance = 6.0;  // inches 24.0 (we've been using 3.0)
    
    //Swerve Speed Constants
    public static final double kSwerveDriveMaxSpeed = 22000.0;
    public static final double kSwerveMaxSpeedInchesPerSecond = 12.5 * 12.0;
    public static final double kSwerveRotationMaxSpeed = 12720.0 * 0.8; //The 0.8 is to request a speed that is always achievable
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationSpeedScalar = ((1.0 / 0.125) - 1.0) / kSwerveMaxSpeedInchesPerSecond;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
    /**
    * To Zero: Rotate module so that bevel gear is face out. Rotate module 90° CW from the top
    * Enter angle read by the absolute encoder. Insert as degrees and subtract or add 90° to the value
    * based on where the bevel ended up.
    */
    public static final double kFrontRightEncoderStartingPos = Settings.kIsUsingCompBot ? -322.2 : -186.455; //Module 0 - Front Right -3.52 | -7.91 | 5.71
    public static final double kFrontLeftEncoderStartingPos = Settings.kIsUsingCompBot ? -141.5 : -137.695; //Module 1 - Front Left -109.97 | -109.95
    public static final double kRearLeftEncoderStartingPos = Settings.kIsUsingCompBot ? -22.64 : -255.376; //Module 2 - Rear Left -219.4 | -219.28 | 242.94
    public static final double kRearRightEncoderStartingPos = Settings.kIsUsingCompBot ? -12.54 : -61.306; //Module 3 - Rear Right -353.5 | 351.77
    
    //Swerve Module Positions (relative to the center of the drive base)
    public static final Translation2d kVehicleToModuleZero = new Translation2d(kWheelbaseLength / 2, kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleOne = new Translation2d(kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleTwo = new Translation2d(-kWheelbaseLength / 2, -kWheelbaseWidth / 2);
    public static final Translation2d kVehicleToModuleThree = new Translation2d(-kWheelbaseLength / 2, kWheelbaseWidth / 2);
    
    public static final List<Translation2d> kModulePositions = Arrays.asList(kVehicleToModuleZero,
    kVehicleToModuleOne, kVehicleToModuleTwo, kVehicleToModuleThree);
    
    //Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
    public static final double kXScrubFactor = 1.0;//1.0 / (1.0 - (9549.0 / 293093.0));
    public static final double kYScrubFactor = 1.0;//1.0 / (1.0 - (4.4736 / 119.9336));
    
    //Voltage-Velocity equation constants {m, b, x-intercept}
    //First set is the positive direction, second set is negative
    public static final double[][][] kVoltageVelocityEquations = new double[][][]{
        {{1.70, -4.39, 2.58}, {1.83, 5.23, -2.85}},
        {{1.59, -3.86, 2.42}, {1.43, 3.09, -2.16}},
        {{1.53, -3.66, 2.39}, {1.66, 4.15, -2.50}},
        {{1.84, -4.70, 2.56}, {1.85, 5.34, -2.89}}
    };
    
    //Swerve Odometry Constants
    public static final double kSwerveWheelDiameter = 4.0587; //inches (actual diamter is closer to 3.87, but secondary algorithm prefers 4.0901) 3.76
    public static final double kSwerveDriveEncoderResolution = 2048.0; //2048.0 for falcon 500
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    /** The number of rotations the swerve rotation motor undergoes for every rotation of the module. */
    public static final double kSwerveRotationReduction = 12.0;
    /** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
    public static final double kSwerveEncoderToWheelRatio = 7.29;
    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    
    public static class Turret {
        
        public static final double kMaxCurrent = 30.0;
        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;
        public static final double kStartingAngle = 0.0; // Turret facing straight forward
        public static final double kFalconToTurretRatio = 70.0; // Falcon Encoder : Turret - Ratio
        public static final double kFalconToEncoderRatio = 0.0;
        public static final double kAngleTolerance = 1.0;
        public static final double kMinControlAngle = -90.0;
        public static final double kMaxControlAngle = 90.0;
        public static final double kMinInitialAngle = -135.0;
        public static final double kMaxInitialAngle = 30.0;

        public static final double kTrackingOffset = 2.0; //3.0 = Close
        
        public static final double kP = 0.25;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023.0 / kMaxSpeed;
        
        // Turret pose with respect to the robot's center
        public static final double kXOffset = -4.9;
        public static final double kYOffset = 0.0;
    
    }
    
    
    public static class LEDs {
        
        //LED Colors
        public static final List<Double> pink = Arrays.asList(255.0, 20.0, 30.0);
        public static final List<Double> blue = Arrays.asList(0.0, 0.0, 255.0);
        public static final List<Double> red = Arrays.asList(255.0, 0.0, 0.0);
        public static final List<Double> orange = Arrays.asList(255.0, 20.0, 0.0);
        public static final List<Double> yellow = Arrays.asList(255.0, 60.0, 0.0);
        public static final List<Double> green = Arrays.asList(0.0, 255.0, 0.0);
        public static final List<Double> purple = Arrays.asList(255.0, 0.0, 255.0);
        
        //LED Arrays
        public static final List<List<Double>> rainbow = Arrays.asList(red, orange, yellow, green, blue, pink, purple);
        
    }
    
    
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kBottomShooterTreeMap = new InterpolatingTreeMap<>();
    static {
        // Key: Distance (inches), Value: RPM
        kBottomShooterTreeMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(2300.0));
        kBottomShooterTreeMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(2850.0));
        kBottomShooterTreeMap.put(new InterpolatingDouble(200.0), new InterpolatingDouble(2650.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kTopShooterTreeMap = new InterpolatingTreeMap<>();
    static {
        // Key: Distance (inches), Value: RPM
        kTopShooterTreeMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(1150.0));
        kTopShooterTreeMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(1425.0));
        kTopShooterTreeMap.put(new InterpolatingDouble(200.0), new InterpolatingDouble(2650.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToHorizontalVelocity = new InterpolatingTreeMap<>();
    static {
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(100.0), new InterpolatingDouble(110.163));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(160.0), new InterpolatingDouble(124.458));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHorizontalVelocityToBottomRPM = new InterpolatingTreeMap<>();
    static {
        kHorizontalVelocityToBottomRPM.put(new InterpolatingDouble(110.163), new InterpolatingDouble(2300.0));
        kHorizontalVelocityToBottomRPM.put(new InterpolatingDouble(124.458), new InterpolatingDouble(2850.0));
    }
    
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHorizontalVelocityToTopRPM = new InterpolatingTreeMap<>();
    static {
        kHorizontalVelocityToTopRPM.put(new InterpolatingDouble(110.163), new InterpolatingDouble(1150.0));
        kHorizontalVelocityToTopRPM.put(new InterpolatingDouble(124.458), new InterpolatingDouble(1425.0));
    }
    
    public static class Shooter{
        public static final double kTopEncToOutputRatio = 36.0 / 16.0;
        public static final double kBottomEncToOutputRatio = 1.0;
        public static final double kShooterRamp = 1.0; //2.0
        
        //Shooter RPM 
        public static final double kCloseTopRPM = 875.0;
        public static final double kCloseBottomRPM = 2200.0;

        public static final double kMidTopRPM = 1150.0; //1100, 3300, 55 deg
        public static final double kMidBottomRPM = 3100.0; //1500, 3300, 53.5 deg

        public static final double kFarTopRPM = 1135.0; //1400, 3600, 63 deg
        public static final double kFarBottomRPM = 4000.0; //1700, 3400, 61 deg

        public static final double kCornerTopRPM = 1135.0;
        public static final double kCornerBottomRPM = 4000.0;
        
        /**
        * Spin Up
        */
        public static final double kTopP = 0.019; // WCP Single 4in: 0.02
        public static final double kTopI = 0.0001; // WCP Single 4in: 0.0001
        public static final double kTopD = 0.0;
        public static final double kTopF = 0.048; // Theoretical is 0.568 | WCP Single 4in: 0.08
        
        public static final double kBottomP = 0.008; //0.01
        public static final double kBottomI = 0.0001; // 0.0001
        public static final double kBottomD = 0.0;
        public static final double kBottomF = 0.048; // Theoretical is 0.568 | WCP 3 x 4in: 0.048
        /**
        * Hold
        */
        public static final double kTopHoldP = 0.2; // WCP Single 4in: 0.4
        public static final double kTopHoldI = 0.0;
        public static final double kTopHoldD = 1.0; // WCP Single 4in: 6.38
        public static final double kTopHoldF = 0.048; // Value gets changed as soon as it enters hold state so value never gets used3
        
        public static final double kBottomHoldP = 1.0; // WCP 3 x 4in: 1.9 | 1.8
        public static final double kBottomHoldI = 0.0;
        public static final double kBottomHoldD = 5.0; // WCP 3 x 4in: 10.0 | 1.8 | 3.6
        public static final double kBottomHoldF = 0.048; // Value gets changed as soon as it enters hold state so value never gets used
        
        public static final double kShooterRPMTolerance = 150.0;
    }
    
    public static class Intake{
        
        public static final double kIntakeP = 1.0;
        public static final double kIntakeI = 0.0;
        public static final double kIntakeD = 0.0;
        public static final double kIntakeF = 0.0;
        
        public static final double kIntakeSpeed = 0.6;
        public static final double kOuttakeSpeed = -0.5;
        public static final double kFeedingSpeed = 0.5;
        public static final double kHumanLoadSpeed = 0.5;
        public static final double kFastIntakeSpeed = 0.75;
    }
    public static class Wrist {
        public static final double kWristRatio = 0.0; //This value needs to be found.
        public static final double kWristSpeed = 0.5;
        public static final double kWristStartingAngle = 0.0;
        public static final double kWristStartingEncoderPosition = 0.0;
        public static final double kCANCoderToWristRatio = 0.0;
        public static final double kFalconToWristRatio = 112.5;

        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;
        public static final double kMaxInitialAngle = 0.0;
        public static final double kMinInitialAngle = 0.0;


        public static final double kMaxWristAngle = 70.0;
        public static final double kMinWristAngle = -60.0;
        public static final double kWristHardStopAngle = -67.0;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023.0/kMaxSpeed;

        public static final double kIntakeAngle = 57.0;
        public static final double kStowedAngle = -60.0;
        public static final double kBallDebouncerAngle = 25.0;

    }
    public static class Feeder {
        public static final double kFeedingSpeed = 0.9;
        public static final double kReverseSpeed = -0.5;
        public static final double kReceivingSpeed = 0.25;
        
        public static final double kRamp = 0.125;
    }
    
    public static class Hood {
        public static final double kHoodExtensionDelay = 0.25;
    }

    public static class ActuatingHood {
        public static final double kMinHoodAngle = 30.0;
        public static final double kMaxHoodAngle = 70.0;
        public static final double kDeltaAngle = kMaxHoodAngle - kMinHoodAngle;

        public static final double kCloseProtectedAngle = 32.0;
        public static final double kMidHoodAngle = 62.0;

        public static final double kMaxLength = 1.9685;
        public static final double kMinLength = 0.0;
        public static final double kLengthToAngle = 180.0 / 1.9685;
    }

    public static class MotorizedHood {
        public static final double kReduction = 80.89;

        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;

        public static final double kP = 0.1; //Bag: 6.0
        public static final double kI = 0.0;
        public static final double kD = 0.0; //Bag: 32.0
        public static final double kF = 1023.0 / kMaxSpeed;

        public static final double kHoodStartingAngle = 65.0;
        public static final double kEncStartingAngle = Settings.kIsUsingCompBot ? -249.1 : -233.2; // The absolute angle (in degrees) of the mag encoder when the hood is at kHoodStartingAngle

        public static final double kTicksPerDegree = 2048.0 / 360.0 * kReduction;

        public static final double kMinInitialAngle = 10.0;
        public static final double kMaxInitialAngle = 70.0;

        public static final double kMinControlAngle = Settings.kIsUsingCompBot ? 16.0 : 18.0;
        public static final double kMaxControlAngle = 63.0;

        public static final double kAngleTolerance = 2.0;

        //Shooter Hood Angles
        public static final double kCloseAngle = 32.25;
        public static final double kMidAngle = 42.5; //55 deg
        public static final double kFarAngle = 59.0;
    }

    public static class Hanger {
        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;

        public static final double kEncoderReduction = 20.25;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023.0 / kMaxSpeed;

        public static final double kTicksPerInch = 24154.0 / 2.0; //-2.0 -24156 2 inches
        public static final double kHeightTolerance = 2.0;
        
        public static final double kMaxControlHeight = 25.5;
        public static final double kMinControlHeight = 0.0;

        public static final double kMaxInitialHeight = 4.64;//3.64
        public static final double kMinInitialHeight = 2.64;

        public static final double kEncoderStartingAngle = 97.64; //3.433 inches per rotation

        public static final double kManualSpeed = 0.5;
    }

}
