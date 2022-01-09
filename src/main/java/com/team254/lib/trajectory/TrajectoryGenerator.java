package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2020.Constants;
import com.team1323.frc2020.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.CurvatureVelocityConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 120.0;
    private static final double kMaxAccel = 120.0; // 120 
    private static final double kMaxDecel = 72.0; //72
    private static final double kMaxVoltage = 9.0;
    
    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;
    
    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }
    
    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }
    
    public void generateTrajectories() {
        if(mTrajectorySet == null) {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }
    
    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
    boolean reversed,
    final List<Pose2d> waypoints,
    final List<TimingConstraint<Pose2dWithCurvature>> constraints,
    double start_vel,  // inches/s
    double end_vel,  // inches/s
    double max_vel,  // inches/s
    double max_accel,  // inches/s^2
    double max_decel,
    double max_voltage,
    double default_vel,
    int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
        default_vel, slowdown_chunks);
    }
    
    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)
    static final Pose2d autoStartingPose = new Pose2d(Constants.kRobotStartingPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(180.0));
    static final Pose2d partnerStartingPose = new Pose2d(Constants.kPartnerRobotStartingPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(180.0));
    static final Pose2d altAutoStartingPose = new Pose2d(Constants.kAltRobotStartingPose.getTranslation().translateBy(new Translation2d(0.0, 0.0)), Rotation2d.fromDegrees(180.0));

    static final Pose2d firstTrenchIntakePose = Constants.kFirstTrenchBall.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength , 0.0)));
    static final Pose2d thirdTrenchIntakePose = Constants.kLastTrenchBall.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d trenchPairIntakePose = Constants.kTrenchBallSet.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength + 3.0, 0.0)));
    static final Pose2d trenchOppositePairIntakePose = Constants.kOppositeTrenchBalls.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d trenchCornerPose = new Pose2d(new Translation2d( Constants.kFieldLength - 207.57, 107.125), Rotation2d.fromDegrees(-25.0 + 180.0));
    
    static final Pose2d generatorIntake1Pose = Constants.kGeneratorBall1.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d generatorIntake2Pose = Constants.kGeneratorBall2.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));    
    static final Pose2d generatorIntake3Pose = Constants.kGeneratorBall3.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));    
    static final Pose2d generatorIntake4Pose = Constants.kGeneratorBall4.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d generatorIntake5Pose = Constants.kGeneratorBall5.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d generatorIntake1and2Pose = Constants.kGeneratorBall1and2.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));
    static final Pose2d generatorIntake3and4Pose = Constants.kGeneratorBall3and4.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength + 4.0, -9.0)));

    static final Pose2d enemyHumanLoaderIntakePose = Constants.kEnemyHumanLoader.transformBy(Pose2d.fromTranslation(new Translation2d(-Constants.kRobotHalfLength, 0.0)));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left) {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }
            
            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }
            
            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }
        
        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> testPath;
        
        //Preliminary Auto Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToGenerator1;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1ToStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToTrenchRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchRunToStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchRunToCorner;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchRunToCorner10Ball;
        public final Trajectory<TimedState<Pose2dWithCurvature>> poachedTrenchRunToCorner10Ball;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToPoachedTrechRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> poachedTrenchRunToCorner;

        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1ToTrenchRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1ToTrenchCorner;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchCornerToTrenchRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchCornerToPoachedTrenchRun;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToGenerator5;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator5ToStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToEnemyHumanLoader;
        
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToOppoTrench;
        public final Trajectory<TimedState<Pose2dWithCurvature>> oppoTrenchToStart;

        public final Trajectory<TimedState<Pose2dWithCurvature>> altStartToOppoTrench;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToGenerator1and2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1and2ToTrenchCorner;
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToGenerator3and4;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator3and4ToGenerator5;

        public final Trajectory<TimedState<Pose2dWithCurvature>> partnerStartToGenerator1and2;
        public final Trajectory<TimedState<Pose2dWithCurvature>> generator1and2ToStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> modifiedStartToTrenchRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trenchRunToModifiedStart;
        public final Trajectory<TimedState<Pose2dWithCurvature>> modifiedStartToPoachedTrenchRun;
        public final Trajectory<TimedState<Pose2dWithCurvature>> poachedTrenchRunToModifiedStart;
        
        private TrajectorySet() {
            //Test Paths
            testPath = getTestPath();
            
            //Preliminary Auto Paths
            startToGenerator1 = getStartToGenerator1();
            generator1ToStart = getGenerator1ToStart();
            startToTrenchRun = getStartToTrenchRun();
            trenchRunToStart = getTrenchRunToStart();
            trenchRunToCorner = getTrenchRunToCorner();
            trenchRunToCorner10Ball = getTrenchRunToCorner10Ball();
            poachedTrenchRunToCorner10Ball = getPoachedTrenchRunToCorner10Ball();

            startToPoachedTrechRun = getStartToPoachedTrenchRun();
            poachedTrenchRunToCorner = getPoachedTrenchRunToCorner();

            generator1ToTrenchRun = getGenerator1ToTrenchRun();
            generator1ToTrenchCorner = getGenerator1ToTrenchCorner();
            trenchCornerToTrenchRun = getTrenchCornerToTrenchRun();
            trenchCornerToPoachedTrenchRun = getTrenchCornerToPoachedTrenchRun();

            startToGenerator5 = getStartToGenerator5();
            generator5ToStart = getGenerator5ToStart();
            startToEnemyHumanLoader = getStartToEnemyHumanLoader();
            
            startToOppoTrench = getStartToOppoTrench();
            oppoTrenchToStart = getOppoTrenchToStart();

            altStartToOppoTrench = getAltStartToOppoTrench();

            startToGenerator1and2 = getStartToGenerator1and2();
            generator1and2ToTrenchCorner = getGenerator1and2ToTrenchCorner();
            startToGenerator3and4 = getStartToGenerator3and4();
            generator3and4ToGenerator5 = getGenerator3and4ToGenerator5();

            partnerStartToGenerator1and2 = getPartnerStartToGenerator1and2();
            generator1and2ToStart = getGenerator1and2ToStart();
            modifiedStartToTrenchRun = getModifiedStartToTrenchRun();
            trenchRunToModifiedStart = getTrenchRunToModifiedStart();
            modifiedStartToPoachedTrenchRun = getModifiedStartToPoachedTrenchRun();
            poachedTrenchRunToModifiedStart = getPoachRunToModifiedStart();
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTestPath(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(60.0, 0.0)), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPoachRunToModifiedStart(){
            Translation2d adjustmentTranslation = new Translation2d(0.0, -9.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(thirdTrenchIntakePose.getTranslation().x() + adjustmentTranslation.x() - 4.0, thirdTrenchIntakePose.getTranslation().y() + adjustmentTranslation.y()), Rotation2d.fromDegrees(0.0)));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-16.0, 0.0)), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getModifiedStartToPoachedTrenchRun() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, -9.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-16.0, 0.0)), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(firstTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), firstTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(new Translation2d(thirdTrenchIntakePose.getTranslation().x() + adjustmentTranslation.x() - 4.0, thirdTrenchIntakePose.getTranslation().y() + adjustmentTranslation.y()), thirdTrenchIntakePose.getRotation()));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchRunToModifiedStart(){
            Translation2d adjustmentTranslation = new Translation2d(-20.0, -9.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(trenchPairIntakePose.getTranslation().x() + 4.0 + adjustmentTranslation.x(), trenchPairIntakePose.getTranslation().y() + adjustmentTranslation.y()), Rotation2d.fromDegrees(0.0))); //trenchPairIntakePose.getRotation()
            waypoints.add(new Pose2d(firstTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(0.0))); //firstTrenchIntakePose.getRotation()
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-16.0, 0.0)), Rotation2d.fromDegrees(0.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, 80.0, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1and2ToStart() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(generatorIntake1and2Pose.transformBy(Pose2d.fromTranslation(new Translation2d(2.0, -7.0))));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-16.0, 0.0)), Rotation2d.fromDegrees(180.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPartnerStartToGenerator1and2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(partnerStartingPose.getTranslation(), Rotation2d.fromDegrees(180.0)));
            waypoints.add(generatorIntake1and2Pose.transformBy(Pose2d.fromTranslation(new Translation2d(2.0, -7.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToGenerator3and4() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, 0.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-20.0, 0.0)), Rotation2d.fromDegrees(90.0 + 180.0)));
            waypoints.add(new Pose2d(generatorIntake3and4Pose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(-22.5 + 180.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CurvatureVelocityConstraint()), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator3and4ToGenerator5() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, 0.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(generatorIntake3and4Pose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(-112.5)));
            waypoints.add(new Pose2d(generatorIntake5Pose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(-112.5)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CurvatureVelocityConstraint()), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToGenerator5() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, 0.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(6.0, 0.0)), Rotation2d.fromDegrees(90.0 + 180.0)));
            waypoints.add(new Pose2d(generatorIntake3and4Pose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(-150.0)));
            waypoints.add(new Pose2d(generatorIntake5Pose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(-112.5)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CurvatureVelocityConstraint()), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator5ToStart() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, 0.0);

            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(generatorIntake5Pose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(-150.0)));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-20.0, 0.0)), /*autoStartingPose.getRotation()*/ Rotation2d.fromDegrees(-90.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToEnemyHumanLoader() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, -2.0);

            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(adjustmentTranslation), autoStartingPose.getRotation()));
            waypoints.add(new Pose2d(enemyHumanLoaderIntakePose.getTranslation().translateBy(adjustmentTranslation), enemyHumanLoaderIntakePose.getRotation().fromDegrees(180.0)));

            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToPoachedTrenchRun() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, -2.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-50.0 + 180.0)));
            waypoints.add(new Pose2d(firstTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), firstTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(new Translation2d(thirdTrenchIntakePose.getTranslation().x() + 4.0 + adjustmentTranslation.x(), thirdTrenchIntakePose.getTranslation().y() + adjustmentTranslation.y()), thirdTrenchIntakePose.getRotation()));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToGenerator1(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-30.0 + 180.0)));
            waypoints.add(generatorIntake1Pose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToGenerator1and2() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-30.0 + 180.0)));
            waypoints.add(generatorIntake1and2Pose.transformBy(Pose2d.fromTranslation(new Translation2d(2.0, 0.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
            
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1ToStart(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(generatorIntake1Pose);
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-30.0 + 180.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getModifiedStartToTrenchRun(){
            Translation2d adjustmentTranslation = new Translation2d(-18.0, -8.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-16.0, 0.0)), Rotation2d.fromDegrees(-50.0 + 180.0)));
            waypoints.add(new Pose2d(firstTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), firstTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(new Translation2d(trenchPairIntakePose.getTranslation().x() + adjustmentTranslation.x(), trenchPairIntakePose.getTranslation().y() + adjustmentTranslation.y()), trenchPairIntakePose.getRotation()));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, 80.0, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToTrenchRun(){
            Translation2d adjustmentTranslation = new Translation2d(-20.0, -11.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-50.0 + 180.0)));
            waypoints.add(new Pose2d(firstTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), firstTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(new Translation2d(trenchPairIntakePose.getTranslation().x() + 4.0 + adjustmentTranslation.x(), trenchPairIntakePose.getTranslation().y() + adjustmentTranslation.y()), trenchPairIntakePose.getRotation()));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, 80.0, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchRunToStart(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(trenchPairIntakePose);
            waypoints.add(firstTrenchIntakePose);
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(-50.0 + 180.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchRunToCorner(){
            Translation2d adjustmentTranslation = new Translation2d(-20.0, -11.0);
            Translation2d cornerAdjustmentTranslation = new Translation2d(-8.5, -10.5);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(trenchPairIntakePose.getTranslation().translateBy(adjustmentTranslation), trenchPairIntakePose.getRotation()));
            waypoints.add(new Pose2d(thirdTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), thirdTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation().translateBy(cornerAdjustmentTranslation), trenchCornerPose.getRotation()));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPoachedTrenchRunToCorner(){
            Translation2d adjustmentTranslation = new Translation2d(0.0, -2.0);
            Translation2d cornerAdjustmentTranslation = new Translation2d(-4.0, -7.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), thirdTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation().translateBy(cornerAdjustmentTranslation), trenchCornerPose.getRotation()));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchRunToCorner10Ball(){
            Translation2d adjustmentTranslation = new Translation2d(-18.0, -8.0);
            Translation2d cornerAdjustmentTranslation = new Translation2d(-4.0, -12.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(trenchPairIntakePose.getTranslation().translateBy(adjustmentTranslation), trenchPairIntakePose.getRotation()));
            waypoints.add(new Pose2d(thirdTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), thirdTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation().translateBy(cornerAdjustmentTranslation), trenchCornerPose.getRotation()));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getPoachedTrenchRunToCorner10Ball(){
            Translation2d adjustmentTranslation = new Translation2d(0.0, -2.0);
            Translation2d cornerAdjustmentTranslation = new Translation2d(-4.0, -12.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(thirdTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), thirdTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation().translateBy(cornerAdjustmentTranslation), trenchCornerPose.getRotation()));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1ToTrenchRun(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(generatorIntake1Pose.getTranslation(), generatorIntake1Pose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0))));
            waypoints.add(firstTrenchIntakePose);
            waypoints.add(trenchPairIntakePose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1ToTrenchCorner(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(generatorIntake1Pose.getTranslation(), generatorIntake1Pose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0))));
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation(), Rotation2d.fromDegrees(-135.0 + 180.0)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, 60.0, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getGenerator1and2ToTrenchCorner() {
            Translation2d adjustmentTranslation = new Translation2d(-18.0, -24.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(generatorIntake1and2Pose.getTranslation(), generatorIntake1and2Pose.getRotation().rotateBy(Rotation2d.fromDegrees(180.0))));
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(-135.0 + 180.0)));
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, 60, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchCornerToTrenchRun(){
            Translation2d adjustmentTranslation = new Translation2d(-10.0, -6.0);
            Translation2d cornerAdjustmentTranslation = new Translation2d(-18.0, -24.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation().translateBy(cornerAdjustmentTranslation), Rotation2d.fromDegrees(-135.0 + 180.0)));
            waypoints.add(new Pose2d(firstTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), firstTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(trenchPairIntakePose.getTranslation().translateBy(adjustmentTranslation), trenchPairIntakePose.getRotation()));
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CurvatureVelocityConstraint()), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchCornerToPoachedTrenchRun() {
            Translation2d adjustmentTranslation = new Translation2d(0.0, -2.0);
            Translation2d cornerAdjustmentTranslation = new Translation2d(0.0, -10.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(trenchCornerPose.getTranslation().translateBy(cornerAdjustmentTranslation), Rotation2d.fromDegrees(-135.0 + 180.0)));
            waypoints.add(new Pose2d(firstTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), firstTrenchIntakePose.getRotation()));
            waypoints.add(new Pose2d(thirdTrenchIntakePose.getTranslation().translateBy(adjustmentTranslation), thirdTrenchIntakePose.getRotation()));
            
            return generateTrajectory(false, waypoints, Arrays.asList(new CurvatureVelocityConstraint()), kMaxVelocity, 60.0, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToOppoTrench(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(autoStartingPose.getTranslation(), Rotation2d.fromDegrees(90.0 + 180.0)));
            waypoints.add(trenchOppositePairIntakePose);
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getOppoTrenchToStart(){
            Translation2d adjustmentTranslation = new Translation2d(-3.0, 0.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(trenchOppositePairIntakePose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(trenchOppositePairIntakePose.getRotation().getDegrees())));
            waypoints.add(new Pose2d(autoStartingPose.getTranslation().translateBy(new Translation2d(-20.0, 0.0)), Rotation2d.fromDegrees(90.0 + 180.0)));
            
            return generateTrajectory(true, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel, kMaxVoltage, 24.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAltStartToOppoTrench(){
            Translation2d adjustmentTranslation = new Translation2d(-3.0, 0.0);
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(altAutoStartingPose.getTranslation(), Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(trenchOppositePairIntakePose.getTranslation().translateBy(adjustmentTranslation), Rotation2d.fromDegrees(trenchOppositePairIntakePose.getRotation().getDegrees())));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, /*kMaxDecel*/60.0, kMaxVoltage, 24.0, 10);
        }
    }
}
