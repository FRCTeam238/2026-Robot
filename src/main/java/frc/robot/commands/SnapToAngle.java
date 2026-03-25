// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.SnapConstants.*;
import static frc.robot.Constants.DriveConstants.maxVelocityMetersPerSec;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.util;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

@Logged
public class SnapToAngle extends Command {

  private PIDController angleController;
  private Rotation2d angle;
  private boolean firstRun = true;
  /** Creates a new SnapToHub. */
  public SnapToAngle(Rotation2d angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain.getInstance());
    angleController = new PIDController(kP, kI, kD);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Drivetrain.getInstance().setCommand("SnapToAngle");
    if(DriverStation.getAlliance().get() == Alliance.Red && firstRun){
       angle = angle.rotateBy(Rotation2d.k180deg);
       firstRun = false;
    }
    angleController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetRotation = calculateRotationSpeed();
    double[] joyValues = Controls.getInstance().getSwerveJoystickValues();
    Drivetrain.getInstance().driveFromJoysticks(
        joyValues[0] * maxVelocityMetersPerSec,
        joyValues[1] * maxVelocityMetersPerSec,
        targetRotation);
  }

  private double calculateRotationSpeed() {
    Pose2d currentPose = Drivetrain.getInstance().getPose();
    Rotation2d currentRotation = currentPose.getRotation();
    double rotationStep = angleController.calculate(currentRotation.getRadians(), angle.getRadians());
    SmartDashboard.putNumber("speed", rotationStep);
    return rotationStep;
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
    return false;
  }
}
