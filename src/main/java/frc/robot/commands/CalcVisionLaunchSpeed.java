// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LauncherConstants.fuelDetectCurrent;
import static frc.robot.Constants.LauncherConstants.minTime;
import static frc.robot.Constants.LauncherConstants.stopTime;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalcVisionLaunchSpeed extends Command {

  private Timer timer;
  private Timer totalTime;

  /** Creates a new Launch. */
  public CalcVisionLaunchSpeed() {
    addRequirements(Launcher.getInstance(), Feeder.getInstance());
      timer = new Timer();
      totalTime = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Feeder.getInstance().setSpeed(FeederConstants.feederSpeed); //Used for both Intake and Outtake
    Feeder.getInstance().setCommand("LaunchFuel");
    Launcher.getInstance().setCommand("LaunchFuel");
    timer.restart();
    totalTime.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double flywheelSpeed = Launcher.getInstance().calculateLaunchSpeed();
    Launcher.getInstance().setSpeed(flywheelSpeed);
    if (Launcher.getInstance().getStatorCurrent(true) > fuelDetectCurrent){
      timer.reset();
    } else if (Launcher.getInstance().getStatorCurrent(false) > fuelDetectCurrent){
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Feeder.getInstance().stop();
    Launcher.getInstance().stop();
    Feeder.getInstance().setCommand("");
    Launcher.getInstance().setCommand("");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomous() && (timer.get() > stopTime) && (totalTime.get() > minTime);
  }
}
