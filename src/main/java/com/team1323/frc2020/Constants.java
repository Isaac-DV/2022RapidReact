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
    
    public static final double kLooperDt = 0.01;
    public static final double kAutoAimPredictionTime = 0.14; // 0.14

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

    public static final Pose2d autoRightStartingPose = new Pose2d(new Translation2d(297.7142857142857, 92.85714285714286), Rotation2d.fromDegrees(90));
    public static final Pose2d autoLeftStartingPose = new Pose2d(new Translation2d(233.71428571428572, -40.28571428571429), Rotation2d.fromDegrees(-135.0));
    /**
    * Target Specifications
    */
    public static final double kVisionTargetHeight = 104.625; //81.0 to bottom
    public static final double kVisionTargetRadius = 26.6875;
    public static final double kVisionTargetRelativeHeight = 38.25; // From opening of shooter to bottom of goal
    
    //Swerve Calculations Constants (measurements are in inches)
    public static final double kWheelbaseLength = 24.75;
    public static final double kWheelbaseWidth = 24.75;
    public static final double kSwerveDiagonal = Math.hypot(kWheelbaseLength, kWheelbaseWidth);
    
    //Camera Constants (X and Y are with respect to the turret's center)
    public static final double kCameraYOffset = 0.0;//0.25
    public static final double kCameraXOffset = 7.236; //8.5 //9.586
    public static final double kCameraZOffset = 44.467; //26.776 24.524 //42.095
    public static final double kCameraYawAngleDegrees = 0.0;//-12.7
    public static final double kCameraPitchAngleDegrees = Settings.kIsUsingCompBot ? 39.5 : 38.0; //37.5

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
    public static final double kSwerveXInputRate = 0.5;
    public static final double kSwerveYInputRate = 0.5;
    
    //Swerve Module Wheel Offsets (Rotation encoder values when the wheels are facing 0 degrees)
    /**
    * To Zero: Rotate module so that bevel gear is face out. Rotate module 90° CW from the top
    * Enter angle read by the absolute encoder. Insert as degrees and subtract or add 90° to the value
    * based on where the bevel ended up.
    */
    public static final double kFrontRightEncoderStartingPos = Settings.kIsUsingCompBot ? -312.539062 : -188.701172; //Module 0 - Front Right -3.52 | -7.91 | 5.71
    public static final double kFrontLeftEncoderStartingPos = Settings.kIsUsingCompBot ? -17.402344 : -312.275391; //Module 1 - Front Left -109.97 | -109.95
    public static final double kRearLeftEncoderStartingPos = Settings.kIsUsingCompBot ? -218.759766 : -174.462891; //Module 2 - Rear Left -219.4 | -219.28 | 242.94
    public static final double kRearRightEncoderStartingPos = Settings.kIsUsingCompBot ? -199.687500 : -23.818359; //Module 3 - Rear Right -353.5 | 351.77
    
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
    public static final double kSwerveEncoderToWheelRatio = 7.791666667; //7.132867133
    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * kSwerveEncoderToWheelRatio;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    
    
    
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
        public static final double kWristStartingAngle = -19.5;
        public static final double kWristStartingEncoderPosition = (Settings.kIsUsingCompBot) ? 266.444404 : 192.918614;
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

        public static final double kIntakeAngle = 107.0; //96.0
        public static final double kStowedAngle = -10.0;
        public static final double kBallDebouncerAngle = 35.0; //65.0
        public static final double kLowestAngle = 135.0;

    }
    
    public static class BallSplitter {
        public static final double kD2F = 64; //Distance to Flagpoint : The length in which the ball is fired from the ejectors, in inches
        public static final double kWheelDiameter = 4.0;
        public static final double kGameBallDiameter = 9.5;     
        
    }

    public static class BallFeeder {
        public static final double kSplitterRunTime = 1.0;
        public static final double kIntakeAutoRunTime = 1.0;
    }

    public static class Column {
        public static final double kMaxSpeed = 6380.0;

        public static final double kFeedBallSpeed = 1.0;
        public static final double kReverseSpeed = -1.0;

        public static final double kMinFeedDelay = 0.125;
        public static final double kMaxFeedDelay = 0.5;

        public static final double kMinDistance = 60.0;
        public static final double kMaxDistance = 180.0;

        public static final double kFeedVelocitySpeed = kMaxSpeed * 0.1;
        public static final double kQueueVelocitySpeed = kMaxSpeed * 0.5; //0.4
        public static final double kBallDelay = 0.001;

        public static final double kColumnRunTime = 0.5;
    }

    public static class Turret {
        
        public static final double kMaxCurrent = 30.0;
        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;
        public static final double kTurretStartingEncoderPosition = Settings.kIsUsingCompBot ? 107.7 : 349.987509;
        public static final double kTurretStartingAngle = -180.0; // Turret facing straight forward
        public static final double kFalconToTurretRatio = 65.0; // Falcon Encoder : Turret - Ratio
        public static final double kEncoderToTurretRatio = Settings.kIsUsingCompBot ? 2.0 : 1.0;
        public static final double kAngleTolerance = 2.0;
        public static final double kMinControlAngle = -270.0; //-250
        public static final double kMaxControlAngle = 135.0;
        public static final double kMinInitialAngle = Settings.kIsUsingCompBot ? -270.0 : -270.0;
        public static final double kMaxInitialAngle = Settings.kIsUsingCompBot ? -90.0 : 90.0;
        
        public static final double kP = 0.5; // 0.25
        public static final double kI = 0.001;
        public static final double kD = 7.5; //2.5
        public static final double kF = 1023.0 / kMaxSpeed;
        
        // Turret pose with respect to the robot's center
        public static final double kXOffset = 0.0;
        public static final double kYOffset = 0.0;

        public static final double kWrapSettlingTime = 0.75;
    
    }


    public static class MotorizedHood {

        public static final double kReduction = 206.25;
        public static final double kEncoderToHoodRatio = 4;

        public static final double kMaxSpeed = 6380.0 * 2048.0 / 600.0;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 1023.0 / kMaxSpeed;

        public static final double kHoodStartingAngle = 15.0;
        public static final double kEncStartingAngle = Settings.kIsUsingCompBot ? 138.057636 : 138.057636; // The absolute angle (in degrees) of the mag encoder when the hood is at kHoodStartingAngle

        public static final double kTicksPerDegree = 2048.0 / 360.0 * kReduction;

        public static final double kMinInitialAngle = 10.0;
        public static final double kMaxInitialAngle = 50.0;

        public static final double kMinControlAngle = Settings.kIsUsingCompBot ? 15.0 : 15.0;
        public static final double kMaxControlAngle = 45.0;

        // Measured upward from the ground; corresponds to kMinControlAngle
        public static final double kMaxEmpiricalAngle = Settings.kIsUsingCompBot ? 68.9747 + 26.0 : 65.12195 + 23.0;
        public static final double kMinEmpiricalAngle = kMaxEmpiricalAngle - (kMaxControlAngle - kMinControlAngle);

        public static final double kAngleTolerance = 1.0; // degrees

        public static final double kZeroingCurrent = 3.0; // amps
    }

    public static class Shooter{
        public static final double kEncToOutputRatio = 18.0 / 12.0;

        public static final double kBottomToMotorRatio = 12.0 / 18.0; //1 : Falcon = 12 -> Bottom = 18
        public static final double kTopToBottomRatio = 30.0 / 22.0; //24/18 : 22 -> 30

        public static final double kBottomWheelRadius = 2.0; // inches //2
        public static final double kTopWheelRadius = 1.25; // inches //1

        //Shooter RPM 
        public static final double kCloseTopRPM = 875.0;
        public static final double kCloseBottomRPM = 2200.0;

        public static final double kMidTopRPM = 1150.0; //1100, 3300, 55 deg
        public static final double kMidBottomRPM = 3100.0; //1500, 3300, 53.5 deg

        public static final double kFarTopRPM = 1135.0; //1400, 3600, 63 deg
        public static final double kFarBottomRPM = 4000.0; //1700, 3400, 61 deg

        public static final double kPostShotPercentOutput = 0.5;
        public static final double kPostShotRPM = 1500.0;

        public static final double kShooterP = 0.01; // 0.01
        public static final double kShooterI = 0.0; //0.0001 : 0.0 : 0.0 : 0.00005
        public static final double kShooterD = 2.0; //0.1
        public static final double kShooterF = 0.046; //0.048
        

        public static final double kShooterRPMTolerance = 150.0; //50 //150
        public static final double kOnTargetDuration = 0.1;//Theoretical min is 0.1

        public static final double kBallVelocityScrubFactor = Settings.kIsUsingCompBot ? 305.966 / 417.0345 : 290.570415188 / 426.73300175;


        public static final Translation2d closeShotVector = Translation2d.fromPolar(new Rotation2d(Constants.MotorizedHood.kMinControlAngle + 2), 2500.0);
        public static final Translation2d farShotVector = Translation2d.fromPolar(new Rotation2d(Constants.MotorizedHood.kMinControlAngle + 10), 3500.0);
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
    public static class DoubleTelescopes {
        public static final double kMaxSpeed = (6380.0 * 2048.0) / 600;
        public static final double kMinControlHeight = 0.0;
        public static final double kLowestPoint = 0.0;
        public static final double kMaxControlHeight = 29.0;
        public static final double kTicksPerInch = 173180.0 / 17.375; //-3.000000
        
        public static final double kLeftP = 0.1;
        public static final double kLeftI = 0.0;
        public static final double kLeftD = 6.0;
        public static final double kLeftF = 1023.0 / kMaxSpeed;

        public static final double kRightP = 0.1;
        public static final double kRightI = 0.0;
        public static final double kRightD = 0.0;
        public static final double kRightF = 1023.0 / kMaxSpeed;
        public static final double kRobotPitchAngleTolerance = 2.0;
        public static final double kFirstPitchAngle = -43.0;
        public static final double kSecondPitchAngle = 30.0;
        public static final double kHeightTolerance = 2.0;


        //////////////////////////////////////
        // This is how much the telescope goes up to let the other latch onto the next bar.
        public static final double kPreFullReleaseHeight = 0.1;
        /////////////////////////////////////


        public static final double kZeroingCurrent = 3.0;
    }

    public static final double kRPMOffset = -100.0;

    public static InterpolatingTreeMap<InterpolatingDouble, Translation2d> kDistanceToShotVectorMap = new InterpolatingTreeMap<>();
    static {
        // Key: distance from the vision target, in inches
        // Value: a Translation2d whose direction represents a hood angle, and whose magnitude represents a shooter RPM
        kDistanceToShotVectorMap.put(new InterpolatingDouble(60.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 12.0), 1800.0 + kRPMOffset)); //
        kDistanceToShotVectorMap.put(new InterpolatingDouble(72.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 14.0), 1900.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(84.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 15.0), 2000.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(96.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 19.0), 2050.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(108.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 19.0), 2200.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(120.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 23.0), 2000.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(132.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 24.0), 2000.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(144.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 26.0), 2000.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(156.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 2100.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(168.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2200.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(180.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2350.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(192.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2750.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(204.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 2850.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(216.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 27.0), 2885.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(228.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.0), 2950.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(240.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3050.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(252.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3050.0 + kRPMOffset));
        kDistanceToShotVectorMap.put(new InterpolatingDouble(264.0 + kVisionTargetRadius), Translation2d.fromPolar(Rotation2d.fromDegrees(MotorizedHood.kMinControlAngle + 28.5), 3050.0 + kRPMOffset));

    }
}
