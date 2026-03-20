package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import java.util.HashMap;
import java.util.Map;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.BLine.FollowPath;

/** for swerve */
@Logged
public class Drivetrain extends SubsystemBase {

  @NotLogged
  SwerveModule frontLeft = new SwerveModule(frontLeftDriveCANId, frontLeftTurnCANId);
  @NotLogged
  SwerveModule frontRight = new SwerveModule(frontRightDriveCANId, frontRightTurnCANId);
  @NotLogged
  SwerveModule backLeft = new SwerveModule(backLeftDriveCANId, backLeftTurnCANId);
  @NotLogged
  SwerveModule backRight = new SwerveModule(backRightDriveCANId, backRightTurnCANId);

  @NotLogged
  SwerveDrivePoseEstimator odometry;
  Pose2d desiredPose = Pose2d.kZero;
  double lastLinearAccelX = 0;
  double lastLinearAccelY = 0;
  AHRS gyro;
  PIDController x, y, theta;
  String command = "None";
  Field2d field = new Field2d();
  SwerveSample trajectoryPose = new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] { 0, 0, 0, 0 },
      new double[] { 0, 0, 0, 0 });

  @NotLogged private static Drivetrain singleton;
  @NotLogged private static Map<String, DataLogEntry> blineLogging;

  public FollowPath.Builder blineBuilder;

  private Drivetrain() {
    SmartDashboard.putData("field", field);
    gyro = new AHRS(NavXComType.kUSB1);
    odometry = new SwerveDrivePoseEstimator(
        kDriveKinematics,
        gyro.getRotation2d().unaryMinus(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        new Pose2d());
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 15));

    x = new PIDController(kP, kI, kD);
    y = new PIDController(kP, kI, kD);
    theta = new PIDController(kPAngular, kIAngular, kDAngular);
    theta.enableContinuousInput(-Math.PI, Math.PI);

    configureBLine();
  }

  public static Drivetrain getInstance() {
    if (singleton == null)
      singleton = new Drivetrain();
    return singleton;
  }

  @Override
  public void periodic() {
    odometry.update(
        gyro.getRotation2d().unaryMinus(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    field.setRobotPose(getPose());

    if (Robot.enableVision) {
      Vision.getInstance().runVision();
    }
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void setCommand(String name) {
    command = name;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d().unaryMinus(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  public Rotation2d getFieldRelativeOffset() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        return odometry.getEstimatedPosition().getRotation();
      } else {
        return odometry
            .getEstimatedPosition()
            .getRotation()
            .minus(Rotation2d.fromDegrees(180)); // DO NOT USE IF WE DONT RUN A PATH
      }
    } else {
      return odometry.getEstimatedPosition().getRotation();
    }
  }

  public void driveFromJoysticks(double xSpeed, double ySpeed, double rot) {

    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, getFieldRelativeOffset())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    setModuleStates(swerveModuleStates);
  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, odometry.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            .02));
    setModuleStates(swerveModuleStates);
  }

  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, .02));
    setModuleStates(swerveModuleStates);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocityMetersPerSec);
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  @Logged
  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState(),
    };
  }

  @Logged
  public SwerveModuleState[] getDesiredStates() {
    return new SwerveModuleState[] {
        frontLeft.getDesiredState(), frontRight.getDesiredState(), backLeft.getDesiredState(),
        backRight.getDesiredState()
    };
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition(),
    };
  }

  public void zeroHeading() {
    // gyro.reset();
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        // gyro.setAngleAdjustment(180);
        odometry.resetRotation(new Rotation2d(Math.PI));
      } else {
        odometry.resetRotation(new Rotation2d(0));
        // gyro.setAngleAdjustment(0);
      }
    }
  }

  public Command zeroHeadingCommand() {
    return Commands.runOnce(
        () -> {
          this.zeroHeading();
        });
  }

  public double getHeading() {
    return gyro.getRotation2d().unaryMinus().getDegrees();
  }

  public double getTurnRate() {
    return -1*gyro.getRate();
  }

  public void setDesiredPose(Pose2d pose) {
    desiredPose = pose;
  }
  
  public void stop () {
        driveFieldRelative(0.0,0.0,0.0);
    }

  public void configureBLine()
  {
    blineBuilder = new FollowPath.Builder(
      this, 
      this::getPose, 
      this::getChassisSpeeds, 
      this::driveWithChassisSpeeds, 
      new PIDController(translationKP, 0, 0),
      new PIDController(rotationKP, 0, 0),
      new PIDController(crossTrackKP, 0, 0)).withDefaultShouldFlip();
      
    blineLogging = new HashMap<>();
    FollowPath.setDoubleLoggingConsumer(this::blineLoggerDouble);
    FollowPath.setBooleanLoggingConsumer(this::blineLoggerBool);
    FollowPath.setPoseLoggingConsumer(this::blineLoggerPose);
    FollowPath.setTranslationListLoggingConsumer(this::blineLoggerTranslationArray);
  }

  public void blineLoggerBool(Pair<String, Boolean> pair) {
    if(!blineLogging.containsKey(pair.getFirst()))
    {
      blineLogging.put(pair.getFirst(), new BooleanLogEntry(DataLogManager.getLog(), pair.getFirst()));
    }
    ((BooleanLogEntry) blineLogging.get(pair.getFirst())).append(pair.getSecond());
  }

  public void blineLoggerDouble(Pair<String, Double> pair) {
    if(!blineLogging.containsKey(pair.getFirst()))
    {
      blineLogging.put(pair.getFirst(), new DoubleLogEntry(DataLogManager.getLog(), pair.getFirst()));
    }
    ((DoubleLogEntry) blineLogging.get(pair.getFirst())).append(pair.getSecond());
  }

  public void blineLoggerPose(Pair<String, Pose2d> pair) {
    if(!blineLogging.containsKey(pair.getFirst()))
    {
      blineLogging.put(pair.getFirst(), StructLogEntry.create(DataLogManager.getLog(), pair.getFirst(), Pose2d.struct));
    }
    
    ((StructLogEntry<Pose2d>) blineLogging.get(pair.getFirst())).append(pair.getSecond());
  }

  public void blineLoggerTranslationArray(Pair<String, Translation2d[]> pair) {
    if(!blineLogging.containsKey(pair.getFirst()))
    {
      blineLogging.put(pair.getFirst(), StructArrayLogEntry.create(DataLogManager.getLog(), pair.getFirst(), Translation2d.struct));
    }
    
    ((StructArrayLogEntry<Translation2d>) blineLogging.get(pair.getFirst())).append(pair.getSecond());
  }

  public void lockWheels() {

    SwerveModuleState wheelLock[] = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
    };

    setModuleStates(wheelLock);
  }
}
