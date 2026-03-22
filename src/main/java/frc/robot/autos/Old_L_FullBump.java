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
public class Old_L_FullBump extends SequentialCommandGroup {
  /** Creates a new L_FullBump. */
  public Old_L_FullBump() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BLinePath("L_BumpRunnup", false, true),
      new BLinePath("L_Bump", false, false),
      new BLinePath("L_FindInMid", false,  false).deadlineFor(new DeployIntake()),
      new WaitCommand(.5).deadlineFor(new DeployIntake()),
      new BLinePath("L_MidApproach", false, false),
      new BLinePath("L_MidCollect", false, false).deadlineFor(new IntakeFuel()),
      new BLinePath("L_BackFromMid", false, false),
      new SnapToHub().withTimeout(1),
      new CalcLaunchSequence().deadlineFor(new WaitCommand(2.5).andThen(new IntakeMid()))
    );
  }
}
