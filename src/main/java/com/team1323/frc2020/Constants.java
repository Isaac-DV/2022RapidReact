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
    
    public static final double kLooperDt = 0.015;
    public static final double kAutoAimPredictionTime = 0.14;

    public static final double kEpsilon = 0.0001;
    
    //Physical Robot Dimensions (including bumpers)
    public static final double kRobotWidth = 36.5; //35.5
    public static final double kRobotLength = 32.25; //35.5
    public static final double kRobotHalfWidth = kRobotWidth / 2.0;
    public static final double kRobotHalfLength = kRobotLength / 2.0;
    
    public static final double kFieldLength = 629.25;
    
    //Field Landmarks
    public static final Translation2d kCenterOfField = new Translation2d(324.0, 0.0);
    public static final Pose2d kRobotStartingPose = new Pose2d(new Translation2d(kFieldLength - (120.0 + 2.0 + kRobotHalfLength), (161.625 - 94.66)), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kPartnerRobotStartingPose = new Pose2d(new Translation2d(kFieldLength - (120.0 + 2.0 + kRobotHalfLength), (161.625 - 27.75)), Rotation2d.fromDegrees(180.0));
    public static final Pose2d kAltRobotStartingPose = new Pose2d(new Translation2d(kFieldLength - (120.0 + 2.0 + kRobotHalfLength), -(161.625 - 27.75)), Rotation2d.fromDegrees(180.0));
    //!!!ALL ROBOT RELATIVE!!!//
    public static final Pose2d kTopRightQuadrantPose = new Pose2d(new Translation2d(585.0, 105.0), new Rotation2d()); //Opposite Hanger
    public static final Pose2d kTopLeftQuadrantPose = new Pose2d(new Translation2d(602.0, -115.0), new Rotation2d()); //Opposite Terminal
    public static final Pose2d kBottomLeftQuadrantPose = new Pose2d(new Translation2d(61.0, -101.0), new Rotation2d()); //Our alliance's hanger
    public static final Pose2d kBottomRightQuadrantPose = new Pose2d(new Translation2d(47.0, 111.0), new Rotation2d()); //Our alliance's terminal
    public static final List<Pose2d> kFieldCornerPositions = Arrays.asList(kTopRightQuadrantPose, kTopLeftQuadrantPose,
        kBottomLeftQuadrantPose, kBottomRightQuadrantPose);
    /**
    * Target Specifications
    */
    public static final double kVisionTargetHeight = 104.625; //81.0 to bottom
    public static final double kVisionTargetRadius = 26.6875;
    
    //Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 24.75;
    public static final double kWheelbaseWidth = 24.75;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants (X and Y are with respect to the turret's center)
    public static final double kCameraYOffset = 0.0;//0.25
    public static final double kCameraXOffset = 9.586; //8.5
    public static final double kCameraZOffset = 42.095; //26.776 24.524
    public static final double kCameraYawAngleDegrees = 0.0;//-12.7
    public static final double kCameraPitchAngleDegrees = Settings.kIsUsingCompBot ? 37.0 : 37.0; //21.75 for bottom 34.3 37.0604 //39.0

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
    public static final double kFrontRightEncoderStartingPos = Settings.kIsUsingCompBot ? -322.2 : -298.828125; //Module 0 - Front Right -3.52 | -7.91 | 5.71
    public static final double kFrontLeftEncoderStartingPos = Settings.kIsUsingCompBot ? -141.5 : -113.642578; //Module 1 - Front Left -109.97 | -109.95
    public static final double kRearLeftEncoderStartingPos = Settings.kIsUsingCompBot ? -22.64 : -215.948125; //Module 2 - Rear Left -219.4 | -219.28 | 242.94
    public static final double kRearRightEncoderStartingPos = Settings.kIsUsingCompBot ? -12.54 : -189.052734; //Module 3 - Rear Right -353.5 | 351.77
    
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
    public static final double kSwerveRotationReduction = 15.42857143;
    /** The number of rotations the swerve drive encoder undergoes for every rotation of the wheel. */
    public static final double kSwerveEncoderToWheelRatio = 6.538461538;
    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    
    public static class Turret {
        
        public static final double kMaxCurrent = 30.0;
        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;
        public static final double kTurretStartingEncoderPosition = 301.28;
        public static final double kTurretStartingAngle = 0.0; // Turret facing straight forward
        public static final double kFalconToTurretRatio = 65.0; // Falcon Encoder : Turret - Ratio
        public static final double kEncoderToTurretRatio = 1.0;
        public static final double kAngleTolerance = 1.0;
        public static final double kMinControlAngle = -250.0;
        public static final double kMaxControlAngle = 60.0;
        public static final double kMinInitialAngle = -300.0;
        public static final double kMaxInitialAngle = 60.0;
        public static final double kTrackingOffset = 2.0; //3.0 = Close
        
        public static final double kP = 0.25;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023.0 / kMaxSpeed;
        
        // Turret pose with respect to the robot's center
        public static final double kXOffset = 0.0;
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
        kBottomShooterTreeMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(0.0));
        kBottomShooterTreeMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(0.0));
        kBottomShooterTreeMap.put(new InterpolatingDouble(200.0), new InterpolatingDouble(0.0));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kTopShooterTreeMap = new InterpolatingTreeMap<>();
    static {
        // Key: Distance (inches), Value: RPM
        kTopShooterTreeMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(0.0));
        kTopShooterTreeMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(0.0));
        kTopShooterTreeMap.put(new InterpolatingDouble(200.0), new InterpolatingDouble(0.0));
    }
    
    public static class Shooter{
        public static final double kEncToOutputRatio = 1.0;
        public static final double kBottomEncToOutputRatio = 18.0 / 24.0;

        //Shooter RPM 
        public static final double kCloseTopRPM = 875.0;
        public static final double kCloseBottomRPM = 2200.0;

        public static final double kMidTopRPM = 1150.0; //1100, 3300, 55 deg
        public static final double kMidBottomRPM = 3100.0; //1500, 3300, 53.5 deg

        public static final double kFarTopRPM = 1135.0; //1400, 3600, 63 deg
        public static final double kFarBottomRPM = 4000.0; //1700, 3400, 61 deg

        public static final double kShooterP = 1.0; //0.15
        public static final double kShooterI = 0.000;
        public static final double kShooterD = 0.0;
        public static final double kShooterF = 0.051; //0.051

        public static final double kShooterRPMTolerance = 150.0;

        public static final double kBallVelocityScrubFactor = 310.5153 / 418.879;
    }
    
    public static class Intake{
        
       
        
        public static final double kIntakeSpeed = 1.0;
        public static final double kOuttakeSpeed = -0.5;
        public static final double kFeedingSpeed = 0.5;
        public static final double kHumanLoadSpeed = 0.5;
        public static final double kFastIntakeSpeed = 0.75;
    }
    public static class Wrist {
        public static final double kWristRatio = 0.0; //This value needs to be found.
        public static final double kWristSpeed = 0.5;
        public static final double kWristStartingAngle = -25.47;
        public static final double kWristStartingEncoderPosition = 178.654306;
        public static final double kCANCoderToWristRatio = 22.0 / 12.0;
        public static final double kFalconToWristRatio = 66.0;

        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;
        public static final double kMaxInitialAngle = 165.0;
        public static final double kMinInitialAngle = -30.0;


        public static final double kMaxWristAngle = 135.0;
        public static final double kMinWristAngle = -15.0;
        public static final double kWristHardStopAngle = -25.47;

        public static final double kP = 0.145;
        public static final double kI = 0.0;
        public static final double kD = 12.5;
        public static final double kF = 1023.0/kMaxSpeed;

        public static final double kIntakeAngle = 98.0;
        public static final double kStowedAngle = -10.0;
        public static final double kBallDebouncerAngle = 65.0;

    }
    public static class Feeder {
        public static final double kFeedingSpeed = 0.9;
        public static final double kReverseSpeed = -0.5;
        public static final double kReceivingSpeed = 0.25;
        
        public static final double kRamp = 0.125;
    }
    public static class Column {
        public static final double kFeedBallSpeed = 0.75;
        public static final double kReverseSpeed = -1.0;
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

    public static class Telescope {
        public static final double kMaxSpeed = (6380.0 * 2048.0) / 600;
        public static final double kTicksPerInch = 0;

        public static final double kMinControlHeight = 0.5;
        public static final double kMaxControlHeight = 10.0;
        public static final double kHeightTolerance = 0.2;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;
    }
    public static class Elevator {
        public static final double kMaxSpeed = (6380.0 * 2048.0) / 600;
        public static final double kFalconTicksPerInch = 150355.0 / 10.25;
        public static final double kFalconToMagEncoderRatio = 45.0;
        public static final double kMagEncoderStartingPosition = -0.319385;
        public static final double kStartingHeight = 6.25;
        public static final double kMinInitialHeight = -1.0; // 6.5 inches before encoder wrap
        public static final double kMaxInitialHeight = 5.5;

        public static final double kMinControlHeight = 0.5;
        public static final double kMaxControlHeight = 56.0;
        public static final double kHeightTolerance = 0.2;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023.0 / kMaxSpeed;
    }
    public static class ClawWrist {
        public static final double kMaxSpeed = (6380.0 * 2048.0) / 600;
        public static final double kMinControlAngle = 0.0;
        public static final double kMaxControlAngle = 100.0;
        public static final double kMinInitialAngle = 0.0;
        public static final double kMaxInitialAngle = 0.0;
        public static final double kFalconToWristRatio = 1096.875;
        public static final double kFalconToCANCoderRatio = 0.0;
        public static final double kWristAngleTolerance = 2.0;
        
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023.0 / kMaxSpeed;
    }
    public static class MotorizedHood {
        public static final double kMinControlAngle = 27.0; //Lowest Hood physical Angle = 27deg
        public static final double kMaxControlAngle = 42.5; //Highest Hood physical angle = 42.5deg
        public static final double kServoAngleToHoodHeight = 2.0 / 180; //Not the real values 

        // Measured upward from the ground; corresponds to kMinControlAngle
        public static final double kMaxEmpiricalAngle = 69.5172;

        public static final double kServoSpeed = 0.5; // Stroke percentage / second

        public static final double kAngleTolerance = 1.0; // degrees
    }
    public static class BallSplitter {
        public static final double kEjectorLength = 64; //The length in which the ball is fired from the ejectors, in inches
        
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToHorizontalVelocity = new InterpolatingTreeMap<>();
    static {
        // Key: Distance (inches), Value: Horizontal Velocity (inches/sec)
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(45.0), new InterpolatingDouble(83.802));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(47.0), new InterpolatingDouble(84.526));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(49.0), new InterpolatingDouble(85.432));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(51.0), new InterpolatingDouble(86.156));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(53.0), new InterpolatingDouble(87.107));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(55.0), new InterpolatingDouble(87.741));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(57.0), new InterpolatingDouble(88.511));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(59.0), new InterpolatingDouble(89.371));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(61.0), new InterpolatingDouble(90.050));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(63.0), new InterpolatingDouble(90.865));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(65.0), new InterpolatingDouble(91.725));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(67.0), new InterpolatingDouble(92.359));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(69.0), new InterpolatingDouble(93.083));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(71.0), new InterpolatingDouble(94.079));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(73.0), new InterpolatingDouble(94.668));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(75.0), new InterpolatingDouble(95.347));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(77.0), new InterpolatingDouble(96.026));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(79.0), new InterpolatingDouble(97.067));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(81.0), new InterpolatingDouble(97.656));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(83.0), new InterpolatingDouble(98.290));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(85.0), new InterpolatingDouble(98.969));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(87.0), new InterpolatingDouble(99.648));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(89.0), new InterpolatingDouble(100.327));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(91.0), new InterpolatingDouble(101.368));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(93.0), new InterpolatingDouble(101.957));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(95.0), new InterpolatingDouble(102.591));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(97.0), new InterpolatingDouble(103.225));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(99.0), new InterpolatingDouble(103.858));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(101.0), new InterpolatingDouble(104.538));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(103.0), new InterpolatingDouble(105.171));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(105.0), new InterpolatingDouble(105.850));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(107.0), new InterpolatingDouble(106.484));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(109.0), new InterpolatingDouble(107.163));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(111.0), new InterpolatingDouble(107.843));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(113.0), new InterpolatingDouble(108.522));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(115.0), new InterpolatingDouble(109.201));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(117.0), new InterpolatingDouble(109.880));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(119.0), new InterpolatingDouble(110.876));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(121.0), new InterpolatingDouble(111.510));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(123.0), new InterpolatingDouble(112.144));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(125.0), new InterpolatingDouble(112.777));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(127.0), new InterpolatingDouble(113.456));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(129.0), new InterpolatingDouble(113.638));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(131.0), new InterpolatingDouble(114.271));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(133.0), new InterpolatingDouble(114.860));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(135.0), new InterpolatingDouble(115.494));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(137.0), new InterpolatingDouble(116.082));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(139.0), new InterpolatingDouble(116.716));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(141.0), new InterpolatingDouble(117.350));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(143.0), new InterpolatingDouble(117.984));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(145.0), new InterpolatingDouble(118.572));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(147.0), new InterpolatingDouble(119.206));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(149.0), new InterpolatingDouble(119.885));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(151.0), new InterpolatingDouble(120.519));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(153.0), new InterpolatingDouble(121.198));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(155.0), new InterpolatingDouble(121.877));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(159.0), new InterpolatingDouble(122.692));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(161.0), new InterpolatingDouble(123.281));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(163.0), new InterpolatingDouble(123.869));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(165.0), new InterpolatingDouble(124.503));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(167.0), new InterpolatingDouble(125.137));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(169.0), new InterpolatingDouble(125.771));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(171.0), new InterpolatingDouble(126.495));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(175.0), new InterpolatingDouble(127.265));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(177.0), new InterpolatingDouble(127.854));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(179.0), new InterpolatingDouble(128.442));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(181.0), new InterpolatingDouble(129.121));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(183.0), new InterpolatingDouble(129.800));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(187.0), new InterpolatingDouble(130.615));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(189.0), new InterpolatingDouble(131.159));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(191.0), new InterpolatingDouble(131.792));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(193.0), new InterpolatingDouble(132.426));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(195.0), new InterpolatingDouble(133.151));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(199.0), new InterpolatingDouble(133.875));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(201.0), new InterpolatingDouble(134.464));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(203.0), new InterpolatingDouble(135.097));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(205.0), new InterpolatingDouble(135.867));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(207.0), new InterpolatingDouble(136.048));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(209.0), new InterpolatingDouble(136.546));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(211.0), new InterpolatingDouble(137.135));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(213.0), new InterpolatingDouble(137.814));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(217.0), new InterpolatingDouble(138.584));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(219.0), new InterpolatingDouble(139.172));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(221.0), new InterpolatingDouble(139.806));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(225.0), new InterpolatingDouble(140.666));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(227.0), new InterpolatingDouble(141.209));
        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(229.0), new InterpolatingDouble(141.843));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHorizontalVelocityToRPM = new InterpolatingTreeMap<>();
    static {
        // Key: Horizontal Velocity (inches/sec), Value: Shooter RPM
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(88.0), new InterpolatingDouble(1942.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(89.0), new InterpolatingDouble(1964.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(90.0), new InterpolatingDouble(1986.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(91.0), new InterpolatingDouble(2008.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(92.0), new InterpolatingDouble(2030.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(93.0), new InterpolatingDouble(2052.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(94.0), new InterpolatingDouble(2075.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(95.0), new InterpolatingDouble(2097.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(96.0), new InterpolatingDouble(2119.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(97.0), new InterpolatingDouble(2141.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(98.0), new InterpolatingDouble(2163.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(99.0), new InterpolatingDouble(2185.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(100.0), new InterpolatingDouble(2207.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(101.0), new InterpolatingDouble(2229.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(102.0), new InterpolatingDouble(2251.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(103.0), new InterpolatingDouble(2273.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(104.0), new InterpolatingDouble(2295.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(105.0), new InterpolatingDouble(2318.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(106.0), new InterpolatingDouble(2340.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(107.0), new InterpolatingDouble(2362.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(108.0), new InterpolatingDouble(2384.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(109.0), new InterpolatingDouble(2406.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(110.0), new InterpolatingDouble(2428.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(111.0), new InterpolatingDouble(2450.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(112.0), new InterpolatingDouble(2472.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(113.0), new InterpolatingDouble(2494.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(114.0), new InterpolatingDouble(2516.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(115.0), new InterpolatingDouble(2538.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(116.0), new InterpolatingDouble(2560.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(117.0), new InterpolatingDouble(2583.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(118.0), new InterpolatingDouble(2605.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(119.0), new InterpolatingDouble(2627.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(120.0), new InterpolatingDouble(2649.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(121.0), new InterpolatingDouble(2671.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(122.0), new InterpolatingDouble(2693.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(123.0), new InterpolatingDouble(2715.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(124.0), new InterpolatingDouble(2737.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(125.0), new InterpolatingDouble(2759.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(126.0), new InterpolatingDouble(2781.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(127.0), new InterpolatingDouble(2803.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(128.0), new InterpolatingDouble(2826.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(129.0), new InterpolatingDouble(2848.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(130.0), new InterpolatingDouble(2870.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(131.0), new InterpolatingDouble(2892.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(132.0), new InterpolatingDouble(2914.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(133.0), new InterpolatingDouble(2936.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(134.0), new InterpolatingDouble(2958.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(135.0), new InterpolatingDouble(2980.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(136.0), new InterpolatingDouble(3002.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(137.0), new InterpolatingDouble(3024.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(138.0), new InterpolatingDouble(3046.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(139.0), new InterpolatingDouble(3068.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(140.0), new InterpolatingDouble(3091.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(141.0), new InterpolatingDouble(3113.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(142.0), new InterpolatingDouble(3135.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(143.0), new InterpolatingDouble(3157.000));
        kHorizontalVelocityToRPM.put(new InterpolatingDouble(144.0), new InterpolatingDouble(3179.000));
    }
}
