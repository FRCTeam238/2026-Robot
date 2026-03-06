// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LauncherConstants.kD;
import static frc.robot.Constants.LauncherConstants.kI;
import static frc.robot.Constants.LauncherConstants.kP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToHub extends Command {

  private Translation2d hubPoint;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private boolean finished;

  /** Creates a new SnapToHub. */
  public SnapToHub() {
    addRequirements(Drivetrain.getInstance());
    xController = new PIDController(kP, kI, kD);
    yController = new PIDController(kP, kI, kD);
    rotationController = new PIDController(kP, kI, kD);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hubPoint = util.getHubPoint();
    finished = false;
    xController.setTolerance(0.1); // adjust as needed
    yController.setTolerance(0.1);
    rotationController.setTolerance(0.05);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = Drivetrain.getInstance().getPose();
    double xSpeed = xController.calculate(currentPose.getX(), hubPoint.getX());
    double ySpeed = yController.calculate(currentPose.getY(), hubPoint.getY());
    double targetAngle = Math.atan2(hubPoint.getY() - currentPose.getY(), hubPoint.getX() - currentPose.getX());
    double rotationSpeed = rotationController.calculate(currentPose.getRotation().getRadians(), targetAngle);

    // Optionally clamp speeds to max/min values
    xSpeed = Math.max(Math.min(xSpeed, 1.0), -1.0);
    ySpeed = Math.max(Math.min(ySpeed, 1.0), -1.0);
    rotationSpeed = Math.max(Math.min(rotationSpeed, 1.0), -1.0);

    Drivetrain.getInstance().driveFieldRelative(xSpeed, ySpeed, rotationSpeed);

    finished = xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
    if (finished) {
      Drivetrain.getInstance().stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
