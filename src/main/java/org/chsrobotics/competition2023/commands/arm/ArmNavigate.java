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

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import java.util.List;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.subsystems.Arm;
import org.chsrobotics.lib.commands.TimerCommand;
import org.chsrobotics.lib.math.geometry.Polygon;
import org.chsrobotics.lib.math.geometry.Vector2D;
import org.chsrobotics.lib.models.DoubleJointedArmKinematics;
import org.chsrobotics.lib.trajectory.planning.ConfigurationSpace;
import org.chsrobotics.lib.trajectory.planning.ConfigurationSpace.ConfigurationSpaceDimension;
import org.chsrobotics.lib.trajectory.planning.Dijkstra;
import org.chsrobotics.lib.trajectory.planning.Dijkstra.CostFunction;
import org.chsrobotics.lib.trajectory.planning.LineOfSightPathOptimize;
import org.chsrobotics.lib.util.Node;

public class ArmNavigate extends ParallelCommandGroup {
    private final ConfigurationSpace cSpace =
            new ConfigurationSpace(
                    new ConfigurationSpaceDimension(-Math.PI, Math.PI, false),
                    new ConfigurationSpaceDimension(-Math.PI, Math.PI, false),
                    Polygon.getRectangle(new Vector2D(1, 1), 1, 1));

    private final Node<Vector2D> start = new Node<>(new Vector2D(0, 0));

    private final Node<Vector2D> end = new Node<>(new Vector2D(3, 3));

    private final Node<Vector2D> mid = new Node<>(new Vector2D(2, 0));

    private final Node<Vector2D> other = new Node<Vector2D>(new Vector2D(1.9, 0.6));

    private final Node<Vector2D> tiny = new Node<Vector2D>(new Vector2D(-0.1, -0.1));

    private final TimerCommand timerCommand = new TimerCommand();

    private final DoubleJointedArmKinematics kinematics =
            new DoubleJointedArmKinematics(
                    Constants.SUBSYSTEM.ARM.LOCAL_LENGTH_METERS,
                    Constants.SUBSYSTEM.ARM.DISTAL_LENGTH_METERS);

    public ArmNavigate(Arm arm) {
        start.addConnection(tiny);

        tiny.addConnection(mid);
        tiny.addConnection(other);

        mid.addConnection(end);
        mid.addConnection(other);

        other.addConnection(end);

        class Function implements CostFunction<Vector2D> {
            @Override
            public double evaluate(Vector2D a, Vector2D b) {
                return Math.abs(a.getX() - b.getX() + a.getY() - b.getY());
            }
        }

        var path = Dijkstra.generatePath(List.of(tiny, other, mid), start, end, new Function());

        System.out.println(path);
        System.out.println(Dijkstra.getTotalCost(path, new Function()));

        var improvedPath = LineOfSightPathOptimize.lineOfSightOptimize(cSpace, path);

        System.out.println(improvedPath);
        System.out.println(Dijkstra.getTotalCost(improvedPath, new Function()));

        addCommands(
                timerCommand,
                new ArmSetpointControl(
                        arm, this::getLocalSetpointRadians, this::getDistalSetpointRadians));
    }

    private Vector2D getEESetpoint() {
        return new Vector2D(1, 1);
    }

    private double getLocalSetpointRadians() {
        return kinematics
                .inverseKinematics(getEESetpoint().getX(), getEESetpoint().getY())
                .firstValue()
                .localAngle;
    }

    private double getDistalSetpointRadians() {
        return kinematics
                .inverseKinematics(getEESetpoint().getX(), getEESetpoint().getY())
                .firstValue()
                .distalAngle;
    }
}
