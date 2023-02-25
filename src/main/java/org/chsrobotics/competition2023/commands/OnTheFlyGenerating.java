/**
Copyright 2023 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
*/
package org.chsrobotics.competition2023.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.chsrobotics.competition2023.Localizer;
import org.chsrobotics.competition2023.subsystems.Drivetrain;
import org.chsrobotics.lib.telemetry.Logger;

public class OnTheFlyGenerating extends ParallelCommandGroup {

    private final Drivetrain drivetrain;
    private final Pose2d endPoint;
    private final Pose2d startPoint;

    private final String subdirString = "On_the_fly_generation";
    private final Logger<Double[]> endPointLogger = new Logger<>("endpoint", subdirString);
    private final Logger<Double[]> controlPointLogger = new Logger<>("controlpoint", subdirString);

    public OnTheFlyGenerating(Drivetrain drivetrain, Pose2d endPoint) {

        this.drivetrain = drivetrain;
        this.endPoint = endPoint;
        this.startPoint = Localizer.getInstance().getEstimatedPose();

        PathPlannerTrajectory trajectory =
                PathPlanner.generatePath(
                        new PathConstraints(2, 1.5),
                        new PathPoint(
                                new Translation2d(startPoint.getX(), startPoint.getY()),
                                startPoint.getRotation()),
                        // new PathPoint(new Translation2d(
                        //         startPoint.getX()+((endPoint.getX()-startPoint.getX())/2),
                        //         startPoint.getY()+((endPoint.getY()-startPoint.getY())/2)),
                        //
                        // Rotation2d.fromRadians(startPoint.getRotation().getRadians()+(endPoint.getRotation().getRadians()))
                        //     ),
                        new PathPoint(
                                new Translation2d(endPoint.getX(), endPoint.getY()),
                                endPoint.getRotation()));

        // controlPointLogger.update(
        //     new Double[] {
        //         startPoint.getX()+((endPoint.getX()-startPoint.getX())/2),
        //         startPoint.getY()+((endPoint.getY()-startPoint.getY())/2),
        //         startPoint.getRotation().getRadians()+(endPoint.getRotation().getRadians())
        //     });
        endPointLogger.update(
                new Double[] {
                    endPoint.getX(), endPoint.getY(), endPoint.getRotation().getRadians()
                });

        addCommands(new TrajectoryFollow(drivetrain, trajectory, false));
    }
}
