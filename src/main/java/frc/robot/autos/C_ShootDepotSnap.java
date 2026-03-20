// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BLinePath;
import frc.robot.commands.CalcLaunchSequence;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.IntakeMid;
import frc.robot.commands.SnapToHub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class C_ShootDepotSnap extends SequentialCommandGroup {
  /** Creates a new C_BackAndShoot. */
  public C_ShootDepotSnap() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BLinePath("C_BackAndShoot", false, true).withTimeout(3),
      new IntakeMid(),
      new SnapToHub().withTimeout(1),
      new CalcLaunchSequence().withTimeout(4),
      new BLinePath("C_Depot_Approach", false, false).deadlineFor(new DeployIntake()),
      new WaitCommand(1),
      new BLinePath("Depot_Intake", false,false).deadlineFor(new IntakeFuel()),
      new IntakeFuel().withTimeout(1.25),
      new BLinePath("Depot_To_Shoot", false, false),
      new SnapToHub().withTimeout(1),
      new CalcLaunchSequence().deadlineFor(new WaitCommand(2.5).andThen(new IntakeMid())).withTimeout(5.5),
      new BLinePath("LineUpBump", false, false).deadlineFor(new DeployIntake()),
      new WaitCommand(0.25),
      new BLinePath("DepotOverBump", false, false)
    );
  }
}
