// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.maxAngularVelocityRadsPerSec;
import static frc.robot.Constants.DriveConstants.maxVelocityMetersPerSec;
import static frc.robot.Constants.LauncherConstants.kD;
import static frc.robot.Constants.LauncherConstants.kI;
import static frc.robot.Constants.LauncherConstants.kP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util;
import frc.robot.subsystems.Drivetrain;

/**
 * SnapToHub Command
 *
 * WHAT THIS DOES:
 * When this command is running, the robot will automatically drive toward the
 * hub
 * (a target location on the field) and rotate to face it — without the driver
 * controlling it.
 *
 * HOW IT WORKS:
 * The field is treated like a coordinate grid (X = left/right, Y =
 * forward/backward).
 * We know where the hub is on that grid, and we know where the robot is (from
 * sensors).
 * We use three "PID controllers" — one for X movement, one for Y movement, and
 * one for turning.
 * Each PID controller continuously calculates how fast the robot should move to
 * reach the target.
 *
 * WHAT IS A PID CONTROLLER?
 * A PID controller is a smart feedback loop. It looks at:
 * - How far off you are right now (Proportional)
 * - How long you've been off (Integral)
 * - How fast the error is changing (Derivative)
 * ...and uses those three values to calculate a smooth correction.
 * Think of it like cruise control in a car — it keeps adjusting to hit the
 * target speed.
 *
 * WHEN DOES IT FINISH?
 * The command ends when the robot is within a small acceptable distance and
 * angle of the hub.
 */
public class SnapToHub extends Command {

  // The X,Y coordinates of the hub on the field (set during initialize)
  private Translation2d hubPoint;

  // Three separate PID controllers — one for each axis of movement and one for
  // rotation
  private PIDController xController; // controls left/right movement
  private PIDController yController; // controls forward/backward movement
  private PIDController rotationController; // controls turning to face the hub

  // Tracks whether the robot has successfully reached the hub
  private boolean finished;

  /**
   * Constructor: runs once when the SnapToHub command object is created (usually
   * at robot startup).
   * Think of this like setting up the tools before a job starts.
   */
  public SnapToHub() {
    // Tell the scheduler this command needs exclusive control of the Drivetrain.
    // No other command can drive the robot while this one is running.
    addRequirements(Drivetrain.getInstance());

    // Create the three PID controllers using tuning constants kP, kI, kD
    // (these are numbers that control how aggressively the PID reacts)
    xController = new PIDController(kP, kI, kD);
    yController = new PIDController(kP, kI, kD);
    rotationController = new PIDController(kP, kI, kD);

    // enableContinuousInput tells the rotation controller that angles wrap around.
    // For example, 179 degrees and -179 degrees are actually very close together,
    // so the robot should turn the short way, not all the way around.
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * initialize() runs once when the command is first scheduled (button pressed,
   * auto starts, etc.).
   * This is where we look up the hub's location and reset any state from last
   * time.
   */
  @Override
  public void initialize() {
    // Look up where the hub is on the field (depends on which alliance we're on:
    // Red or Blue)
    hubPoint = util.getHubPoint();

    // Reset the finished flag so the command doesn't immediately stop
    finished = false;

    // Set tolerances — how close is "close enough" to consider the job done?
    // 0.1 meters (~4 inches) for position, 0.05 radians (~3 degrees) for rotation
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    rotationController.setTolerance(0.05);
  }

  /**
   * execute() runs repeatedly (about 50 times per second) while the command is
   * active.
   * This is the main loop: check where we are, figure out how to move, and drive.
   */
  @Override
  public void execute() {
    // Get the robot's current position (X, Y) and angle on the field from
    // odometry/sensors
    Pose2d currentPose = Drivetrain.getInstance().getPose();

    // Update PID setpoints every loop so the robot keeps correcting toward the hub.
    // We do this here (not just in initialize) so that if the robot is bumped,
    // it will recalculate and drive back toward the hub.
    xController.setSetpoint(hubPoint.getX());
    yController.setSetpoint(hubPoint.getY());

    // Calculate the angle the robot needs to face in order to point at the hub.
    //
    // Math.atan2(y, x) is the inverse tangent function that takes both the Y and X
    // differences as separate inputs. It returns the angle (in radians) of the line
    // from the robot to the hub, measured from the positive X axis.
    //
    // We use atan2 instead of plain atan(y/x) because:
    // 1. atan2 works in all four directions (all quadrants of the field)
    // 2. atan2 avoids a divide-by-zero error when the robot is directly above/below
    // the hub
    // 3. atan2 returns the correct sign (+/-) so the robot turns the right way
    double targetAngle = Math.atan2(
        hubPoint.getY() - currentPose.getY(), // vertical distance to hub
        hubPoint.getX() - currentPose.getX()); // horizontal distance to hub
    rotationController.setSetpoint(targetAngle);

    // Ask each PID controller to calculate the correction needed.
    // calculate() takes the current measurement and returns an output between -1.0
    // and 1.0.
    // We multiply by the robot's max speed to convert to real units (meters/sec,
    // radians/sec).
    double xSpeed = xController.calculate(currentPose.getX()) * maxVelocityMetersPerSec;
    double ySpeed = yController.calculate(currentPose.getY()) * maxVelocityMetersPerSec;
    double rotationSpeed = rotationController.calculate(currentPose.getRotation().getRadians())
        * maxAngularVelocityRadsPerSec;

    // Safety clamp: make sure we never command a speed beyond the robot's physical
    // limits.
    // Math.min/max work like guardrails — the value can't go above the max or below
    // the negative max.
    xSpeed = Math.max(Math.min(xSpeed, maxVelocityMetersPerSec), -maxVelocityMetersPerSec);
    ySpeed = Math.max(Math.min(ySpeed, maxVelocityMetersPerSec), -maxVelocityMetersPerSec);
    rotationSpeed = Math.max(Math.min(rotationSpeed, maxAngularVelocityRadsPerSec), -maxAngularVelocityRadsPerSec);

    // Send the three speed values to the drivetrain to actually move the robot
    Drivetrain.getInstance().driveFieldRelative(xSpeed, ySpeed, rotationSpeed);

    // Check if all three controllers are within their tolerances (close enough to
    // done).
    // The && operator means ALL THREE must be satisfied at the same time.
    finished = xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
    if (finished) {
      Drivetrain.getInstance().stop();
    }
  }

  /**
   * end() runs once when the command finishes or is interrupted (e.g. button
   * released).
   * Always stop the robot here as a safety measure.
   */
  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().stop();
  }

  /**
   * isFinished() is checked by the scheduler after every execute() call.
   * If it returns true, the scheduler will call end() and stop this command.
   */
  @Override
  public boolean isFinished() {
    return finished;
  }
}
