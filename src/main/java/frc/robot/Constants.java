package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public class DriveConstants {

        public static final boolean fieldRelative = true;

        public static final int frontRightDriveCANId = 19;
        public static final int frontLeftDriveCANId = 10;
        public static final int backRightDriveCANId = 30;
        public static final int backLeftDriveCANId = 9;

        public static final int frontRightTurnCANId = 18;
        public static final int frontLeftTurnCANId = 11;
        public static final int backRightTurnCANId = 1;
        public static final int backLeftTurnCANId = 8;

        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0.01;

        public static final double kPAngular = 1;
        public static final double kIAngular = 0;
        public static final double kDAngular = 0;

        public static final double maxVelocityMetersPerSec = 4.3;
        public static final double maxAccelerationMetersPerSec2 = 5; // TODO: make this a real number

        public static final double kTrackWidth = Units.inchesToMeters(21.75);
        public static final double kWheelBase = Units.inchesToMeters(21.75);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d((kWheelBase / 2), kTrackWidth / 2),
                new Translation2d((kWheelBase / 2), -kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2), kTrackWidth / 2),
                new Translation2d((-kWheelBase / 2), -kTrackWidth / 2));

        public static double maxAngularVelocityRadsPerSec = 2 * Math.PI;

        public static final double angleTolerance = 5;
        public static final double velocityTolerance = 0.8;
        public static final double positionTolerance = 0.08;
        public static final double xandyvelocityTolerance = 0.08;

        public static final double translationKP = 5.0;
        public static final double rotationKP = 5.0;
        public static final double crossTrackKP = 2.0;
    }

    public class FeederConstants {
        public static int upperMotorID = 5;
        public static int lowerMotorID = 4;
        public static double statorCurrentLimit = 60.0;
        public static double supplyCurrentLimit = 40.0;

        public static double feederSpeed = 10; //Used for both Intake and Outtake, in Voltage
    }

    public class LauncherConstants {
        public static int leftUpID = 7;
        public static int leftLowID = 6;
        public static int rightUpID = 2;
        public static int rightLowID = 3;
        public static double statorCurrentLimit = 80.0;
        public static double supplyCurentLimit = 50;

        // https://tinyurl.com/2026-238-launcher
        public static final double kP = 0.4; // Theoretical 0.46
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kV = 0.13; // Theoretical 0.19
        public static final double kA = 0.51;
        public static final double kS = 0;

        public static final double tolerance = 2.0; // Value is a percentage of requested speed

        public static final double launchSpeedNear = 57; // Value in RPS
        public static final double launchSpeedFar = 67; //Value in RPS         
        public static final double launchSpeedTest = 20; //Value in RPS
    }

    public class IntakeConstants {
        // https://tinyurl.com/2026-238-Intake
        public static int tiltID = 12;
        public static int rollerID = 17;

        public static double rollerStatorCurrentLimit = 50.0;
        public static double rollerSupplyCurrentLimit = 30;
        public static double tiltStatorCurrentLimit = 60.0;
        public static double tiltSupplyCurrentLimit = 20;

        public static final double tiltKP = 3; // ReCalc 123.47
        public static final double tiltKI = 0;
        public static final double tiltKD = 0; // ReCalc 9.05
        public static final double tiltKV = 5.1;
        public static final double tiltKS = 0.1;
        public static final double tiltKA = 0.05;
        public static final double tiltKG = 0.3;
        public static final double tiltCruise = 1.5; // Max velocity is ~116 rev/s.
        public static final double tiltAcceleration = 4;
        public static final double tiltJerk = 20;
        public static final double tiltTolerance = .05;


        // public static final double tiltExpoKV = 5.64;
        // public static final double tiltExpoKA = 0.06;

        public static final double intakePivotRatio = 47.5;
        public static final double intakeDown = 0;
        public static final double intakeMid = .1;
        public static final double intakeQuarters = .265;
        public static final double intakeUp = .33;

        public static final double intakeRollerVoltage = 12; //Used for both Intake and Outtake
    }

    public class OperatorConstants {

        public static double driverJoystickDeadzone = .1;
        public static double xboxControllerDeadzone = .075; // TODO: find good deadzone values for the xbox controllers

        public enum DriveType {
            JOYSTICK,
            XBOX,
        }
    }

    public static class SnapConstants {
        public static final double kP = 5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double snapToleranceAngle = Units.degreesToRadians(2);
        public static final double snapToleranceAngleNarrow = Units.degreesToRadians(1);
    }

    public class VisionConstants {
        public static Translation2d hubRedPoint = new Translation2d(11.901,4.021); //Coordnites in Meters
        public static Translation2d hubBluePoint = new Translation2d(4.637,4.021);
      
        public static boolean filterByDistanceFromOdometryPose = false;

        public static Transform3d backCameraLocation = new Transform3d(-0.2643124, -0.2286, 0.3913632,
                new Rotation3d(0, Units.degreesToRadians(-23), Units.degreesToRadians(180)));
        public static Transform3d frontCameraLocation = new Transform3d(-0.078218, -0.255397, 0.723331, 
                new Rotation3d(0, Units.degreesToRadians(-22.5), 0));
        public static double maxVisionDistanceTolerance = 5;// for the max distance between cam and tag in meters
        public static double maxAmbiguity = 1; // max ambiguity out of 1
        public static double zTolerance = 0.25;
        public static double rollPitchTolerance = Units.degreesToRadians(10);
        public static double visionPoseDiffTolerance = 1; // for the diff between estimated vision pose and odometry in
                                                          // meters
    }
}
