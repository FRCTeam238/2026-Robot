// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.SnapConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

@Logged
public class SnapToHub extends Command {

  private Translation2d hubPoint;
  private PIDController angleController;
  private boolean xLock;

  /** Creates a new SnapToHub. */
  public SnapToHub() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain.getInstance());
    angleController = new PIDController(kP, kI, kD);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Get hub coordinates
    Drivetrain.getInstance().setCommand("SnapToHub");
    hubPoint = util.getHubPoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetRotation = calculateRotationSpeed();

    if(isSnappedToHubNarrow()) {
      Drivetrain.getInstance().lockWheels();
      xLock = true;
    } else if (isSnappedToHubWide() && xLock) {
      Drivetrain.getInstance().lockWheels();
    } else {
      Drivetrain.getInstance().driveFieldRelative(0,0,targetRotation);
      xLock = false;
    }
  }

  private double calculateRotationSpeed() {
    Pose2d currentPose = Drivetrain.getInstance().getPose();
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d deltaToHub = hubPoint.minus(currentTranslation);
    Rotation2d targetRotation = deltaToHub.getAngle();
    Rotation2d currentRotation = currentPose.getRotation();
    double rotationStep = angleController.calculate(currentRotation.getRadians(), targetRotation.getRadians());
    SmartDashboard.putNumber("speed", rotationStep);
    return rotationStep;
  }

  private double getErrorRotation(Pose2d currentPose) {
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d deltaToHub = hubPoint.minus(currentTranslation);
    Rotation2d targetRotation = deltaToHub.getAngle();
    SmartDashboard.putNumber("targetAngle", targetRotation.getRadians());
    Rotation2d currentRotation = currentPose.getRotation();
    SmartDashboard.putNumber("currentAngle", currentRotation.getRadians());
    Rotation2d errorRotation = targetRotation.minus(currentRotation);
    SmartDashboard.putNumber("errorAngle", errorRotation.getRadians());
    return errorRotation.getRadians();
  }
  
  public boolean isSnappedToHubWide() {
    return Math.abs(getErrorRotation(Drivetrain.getInstance().getPose())) <= snapToleranceAngle;
  }

  public boolean isSnappedToHubNarrow() {
    return Math.abs(getErrorRotation(Drivetrain.getInstance().getPose())) <= snapToleranceAngleNarrow;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().setCommand("");
    Drivetrain.getInstance().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomous() && isSnappedToHubWide();
  }
}
