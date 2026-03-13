// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.snapToleranceAngle;
import static frc.robot.Constants.SnapConstants.kD;
import static frc.robot.Constants.SnapConstants.kI;
import static frc.robot.Constants.SnapConstants.kP;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

@Logged
public class SnapToHub extends Command {

  private Translation2d hubPoint;
  private PIDController angleController;

  /** Creates a new SnapToHub. */
  public SnapToHub() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain.getInstance());
    angleController = new PIDController(kP, kI, kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Get hub coordnates
    hubPoint = util.getHubPoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get robot position, calculate rotation
    //Pose2d currentPose = Drivetrain.getInstance().getPose();
    //double xSpeed = getXSpeed(currentPose);
    //double ySpeed = getYSpeed(currentPose);
    double targetRotation = calculateRotationSpeed();
  // getXSpeed and getYSpeed returns distance. How do we translate it to return speed?
    Drivetrain.getInstance().driveFieldRelative(0,0,targetRotation);
    // Assumption: PID returns 0 
    //finished = xSpeed <= snapToleranceDistance && ySpeed <= snapToleranceDistance && targetRotation <= snapToleranceAngle;
    }

  private double getXSpeed(Pose2d currentPose) {
    //get current x
    double currentX = currentPose.getX();

    //get x distance to hub
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d deltaToHub = hubPoint.minus(currentTranslation);
    //double distanceToHub = deltaToHub.getNorm();

    //calc difference
    double xDistanceToHub = deltaToHub.getX(); 
    double xDistanceToRange = xDistanceToHub - Constants.SnapConstants.xRangeToHub;
    //use pid controller to get step
    double xStep = angleController.calculate(currentX, xDistanceToRange);

    //Set X points using PID Controller with speeds
    

    return xStep;
  }

  private double getYSpeed(Pose2d currentPose) {
   //get current y
    double currentY = currentPose.getY();

    //get y distance to hub
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d deltaToHub = hubPoint.minus(currentTranslation);

    //calc difference
    double yDistanceToHub = deltaToHub.getY(); 
    double yDistanceToRange = yDistanceToHub - Constants.SnapConstants.yRangeToHub;
    //use pid controller to get step
    double yStep = angleController.calculate(currentY, yDistanceToRange);

    //Set X points using PID Controller with speeds
    

    return yStep;
  }

  private double calculateRotationSpeed() {
    Pose2d currentPose = Drivetrain.getInstance().getPose();
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d deltaToHub = hubPoint.minus(currentTranslation);
   //double distanceToHub = deltaToHub.getNorm();
    Rotation2d targetRotation = deltaToHub.getAngle();
    Rotation2d currentRotation = currentPose.getRotation();
    // Rotation2d errorRotation = targetRotation.minus(currentRotation);
    double rotationStep = angleController.calculate(currentRotation.getRadians(), targetRotation.getRadians());
    SmartDashboard.putNumber("speed", rotationStep);
    return rotationStep;
  }

  private double getErrorRotation(Pose2d currentPose) {
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d deltaToHub = hubPoint.minus(currentTranslation);
   //double distanceToHub = deltaToHub.getNorm();
    Rotation2d targetRotation = deltaToHub.getAngle();
    SmartDashboard.putNumber("targetAngle", targetRotation.getRadians());
    Rotation2d currentRotation = currentPose.getRotation();
    SmartDashboard.putNumber("currentAngle", currentRotation.getRadians());
    Rotation2d errorRotation = targetRotation.minus(currentRotation);
    SmartDashboard.putNumber("errorAngle", errorRotation.getRadians());
    return errorRotation.getRadians();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getErrorRotation(Drivetrain.getInstance().getPose())) <= snapToleranceAngle;
  }
}
