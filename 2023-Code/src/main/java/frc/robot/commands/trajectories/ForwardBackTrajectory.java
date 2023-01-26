// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trajectories;

import java.util.ArrayList;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.util.PoseTriplet;

/** Add your docs here. */
public class ForwardBackTrajectory {

    public static final ArrayList<PoseTriplet> forwardBackArray = new ArrayList<PoseTriplet>() {
        {
            add(new PoseTriplet(0, 0, 0));
            add(new PoseTriplet(1.5, 0, 0));
            add(new PoseTriplet(0, 1.5, 1.5));
        }
    };

}
