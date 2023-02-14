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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import java.util.List;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.competition2023.util.CSpacePackageLoader.CSpacePackage;
import org.chsrobotics.lib.commands.TimerCommand;
import org.chsrobotics.lib.math.filters.DifferentiatingFilter;
import org.chsrobotics.lib.math.geometry.CardinalSpline;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.trajectory.planning.Dijkstra;
import org.chsrobotics.lib.trajectory.planning.Dijkstra.CostFunction;
import org.chsrobotics.lib.util.Node;

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
        var start = new Node<>(new Vector2D(2, 2.5));

        var holdNode = new Node<>(start.getData());
        var anotherHoldNode = new Node<>(start.getData());

        var mid = new Node<>(new Vector2D(1.5, 0.25));

        var end = new Node<>(new Vector2D(0.6, 0));

        start.addConnection(holdNode);

        holdNode.addConnection(anotherHoldNode);

        anotherHoldNode.addConnection(mid);

        mid.addConnection(end);

        var costFunction =
                new CostFunction<Vector2D>() {
                    @Override
                    public double evaluate(Vector2D a, Vector2D b) {
                        return Math.abs(a.getX() - b.getX() + a.getY() - b.getY());
                    }
                };

        var path =
                Dijkstra.generatePath(
                        List.of(holdNode, anotherHoldNode, mid), start, end, costFunction);

        // var improvedPath = LineOfSightPathOptimize.lineOfSightOptimize(cSpace, path);

        // improvedPath.add(0, holdNode.getData());
        // improvedPath.add(0, anotherHoldNode.getData());

        double unscaledTime = path.size();
        double pathLength = Dijkstra.getTotalCost(path, costFunction);

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
                        path.toArray(new Vector2D[] {}));

        var vis = new Mechanism2d(6, 3);

        vis.getRoot("drivetrainRoot", 2.5, 0.15)
                .append(
                        new MechanismLigament2d(
                                "drivetrain", 1, 0, 10, new Color8Bit(150, 0, 255)));

        var localActual =
                vis.getRoot(
                                "root",
                                3 + Constants.SUBSYSTEM.ARM.ROBOT_TO_ARM.getX(),
                                0.15 + Constants.SUBSYSTEM.ARM.ROBOT_TO_ARM.getZ())
                        .append(
                                new MechanismLigament2d(
                                        "localActual",
                                        Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                                        arm.getLocalAngleRadians()));
        var distalActual =
                localActual.append(
                        new MechanismLigament2d(
                                "distalActual",
                                Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS,
                                arm.getDistalAngleRadians()));

        var localSetpoint =
                vis.getRoot("root", 2, 2)
                        .append(
                                new MechanismLigament2d(
                                        "localSetpoint",
                                        Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                                        getLocalSetpointRadians(),
                                        6,
                                        new Color8Bit(0, 0, 255)));

        var distalSetpoint =
                localSetpoint.append(
                        new MechanismLigament2d(
                                "distalSetpoint",
                                Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS,
                                getDistalSetpointRadians(),
                                6,
                                new Color8Bit(0, 0, 255)));

        HighLevelLogger.getInstance().publishSendable("armVis", vis);

        var updateVis =
                new CommandBase() {
                    @Override
                    public void execute() {
                        localActual.setAngle(new Rotation2d(arm.getLocalAngleRadians()));
                        distalActual.setAngle(new Rotation2d(arm.getDistalAngleRadians()));

                        localSetpoint.setAngle(new Rotation2d(getLocalSetpointRadians()));
                        distalSetpoint.setAngle(new Rotation2d(getDistalSetpointRadians()));
                    }
                };

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
                updateVis,
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
    }

    private double getDistalSetpointRadians() {
        var splineResult = spline.sample(timescale * timerCommand.getTimeElapsed());

        if (splineResult != null) {
            lastDistal = splineResult.getY();
            return splineResult.getY();
        } else {
            return lastDistal;
        }
    }
}
