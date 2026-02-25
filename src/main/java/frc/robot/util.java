// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class util {

    public static Translation2d getHubPoint() {
        //Get my hub constant points
        //Get Red or Blue alliance type
        Optional <Alliance> currentAlliance = DriverStation.getAlliance();
        if(currentAlliance.isPresent() && currentAlliance.get() == Alliance.Blue ) {
            return Constants.VisionConstants.hubBluePoint;
        }
        return Constants.VisionConstants.hubRedPoint;
        //Use info to translate the target point
        
    }
}


