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
package org.chsrobotics.competition2023.commands.arm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.competition2023.util.CSpacePackageLoader.CSpacePackage;
import org.chsrobotics.lib.commands.TimerCommand;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.math.geometry.CardinalSpline;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.chsrobotics.lib.models.DoubleJointedArmKinematics;
import org.chsrobotics.lib.trajectory.planning.CostFunction;
import org.chsrobotics.lib.trajectory.planning.Dijkstra;
import org.chsrobotics.lib.trajectory.planning.LineOfSightPathOptimize;
import org.chsrobotics.lib.util.NodeGraph;

public class ArmNavigate extends ParallelCommandGroup {
    private final TimerCommand timerCommand = new TimerCommand();

    private final CardinalSpline spline;

    private double lastDistal = 0;
    private double lastLocal = 0;

    private final double timescale;

    public ArmNavigate(
            Arm arm,
            CSpacePackage pack,
            double robotRelativeSetpointXMeters,
            double robotRelativeSetpointYMeters) {

        DoubleJointedArmKinematics kinematics =
                new DoubleJointedArmKinematics(
                        Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                        Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS);

        NodeGraph<Vector2D> nodeGraph = pack.nodes;

        var start =
                nodeGraph.createNode(
                        new Vector2D(arm.getLocalAngleRadians(), arm.getDistalAngleRadians()));

        // TODO: connect start and end nodes to graph

        var sol =
                kinematics.inverseKinematics(
                        robotRelativeSetpointXMeters, robotRelativeSetpointYMeters);

        var leftyEnd =
                nodeGraph.createNode(
                        new Vector2D(sol.firstValue().localAngle, sol.firstValue().distalAngle));

        var rightyEnd =
                nodeGraph.createNode(
                        new Vector2D(sol.secondValue().localAngle, sol.secondValue().distalAngle));

        var costFunction =
                new CostFunction<Vector2D>() {
                    @Override
                    public double evaluate(Vector2D a, Vector2D b) {
                        return Math.abs(a.getX() - b.getX() + a.getY() - b.getY());
                    }
                };

        var leftyPath = Dijkstra.generatePath(nodeGraph, start, leftyEnd, costFunction);
        var rightyPath = Dijkstra.generatePath(nodeGraph, start, rightyEnd, costFunction);

        var improvedLeftyPath = LineOfSightPathOptimize.lineOfSightOptimize(pack.cSpace, leftyPath);

        var improvedRightyPath =
                LineOfSightPathOptimize.lineOfSightOptimize(pack.cSpace, rightyPath);

        var finalPath =
                (Dijkstra.getTotalCost(improvedLeftyPath, costFunction)
                                > Dijkstra.getTotalCost(improvedRightyPath, costFunction))
                        ? improvedRightyPath
                        : improvedLeftyPath;

        double unscaledTime = finalPath.size();
        double pathLength = Dijkstra.getTotalCost(finalPath, costFunction);

        if (pathLength == 0) timescale = 1;
        else
            timescale =
                    (Constants.COMMAND.ARM_NAVIGATE.ARM_NAVIGATE_TIME_SCALING * unscaledTime)
                            / pathLength;

        spline =
                new CardinalSpline(
                        Constants.COMMAND.ARM_NAVIGATE.SPLINE_TENSION,
                        new Rotation3d(),
                        new Rotation3d(),
                        finalPath.toArray(new Vector2D[] {}));

        var splineXVelFilter = new DifferentiatingFilter();
        var splineXAccelFilter = new DifferentiatingFilter();

        var splineYVelFilter = new DifferentiatingFilter();
        var splineYAccelFilter = new DifferentiatingFilter();

        var diffHandler =
                new CommandBase() {
                    @Override
                    public void execute() {
                        splineXVelFilter.calculate(getLocalSetpointRadians());
                        splineXAccelFilter.calculate(splineXVelFilter.getCurrentOutput());

                        splineYVelFilter.calculate(getDistalSetpointRadians());
                        splineYAccelFilter.calculate(splineYVelFilter.getCurrentOutput());
                    }
                };

        addCommands(
                new UpdateArmVis(
                        arm, this::getLocalSetpointRadians, this::getDistalSetpointRadians),
                diffHandler,
                timerCommand,
                new ArmSetpointControl(
                        arm,
                        this::getLocalSetpointRadians,
                        this::getDistalSetpointRadians,
                        splineXVelFilter::getCurrentOutput,
                        splineYVelFilter::getCurrentOutput,
                        splineXAccelFilter::getCurrentOutput,
                        splineYAccelFilter::getCurrentOutput));
    }

    private double getLocalSetpointRadians() {
        var splineResult = spline.sample(timescale * timerCommand.getTimeElapsed());

        if (splineResult != null) {
            lastLocal = splineResult.getX();
            return splineResult.getX();
        } else {
            return lastLocal;
        }
        // return 0;
    }

    private double getDistalSetpointRadians() {
        var splineResult = spline.sample(timescale * timerCommand.getTimeElapsed());

        if (splineResult != null) {
            lastDistal = splineResult.getY();
            return splineResult.getY();
        } else {
            return lastDistal;
        }

        // return 0;
    }
}
