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
package org.chsrobotics.competition2023.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Constants.SUBSYSTEM.VISION;
import org.chsrobotics.competition2023.Localizer;
import org.chsrobotics.lib.telemetry.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision implements Subsystem {
    private static final Vision instance = new Vision();

    private final PhotonCamera cameraA = new PhotonCamera(VISION.CAMERA_A_NAME);

    private final RobotPoseEstimator poseSolver =
            new RobotPoseEstimator(
                    Constants.GLOBAL.TAG_LAYOUT,
                    VISION.DEFAULT_POSE_STRATEGY,
                    List.of(Pair.of(cameraA, VISION.ROBOT_TO_CAMERA_A)));

    private Pair<Pose3d, Double> estimatedPose = Pair.of(new Pose3d(), 0.0);

    private final SimVisionSystem simCameraA =
            new SimVisionSystem(
                    VISION.CAMERA_A_NAME,
                    VISION.CAMERA_A_DIAG_FOV_DEGREES,
                    VISION.ROBOT_TO_CAMERA_A.inverse(),
                    9970,
                    VISION.CAMERA_A_HORIZONTAL_RESOLUTION_PX,
                    VISION.CAMERA_A_VERTICAL_RESOLUTION_PX,
                    VISION.CAMERA_A_SIMULATION_MIN_TARGET_AREA);

    private final List<SimVisionTarget> simTargets = new ArrayList<>();

    private Pose3d simPose = new Pose3d();

    private final String subdirString = "vision";

    private final Logger<Boolean> aHasTargetsLogger =
            new Logger<>("cameraAHasTargets", subdirString);

    private final Logger<Double> aLatencyLogger =
            new Logger<>("cameraAPipelineLatency_s", subdirString);

    private final Logger<Double[]> aTargetCornersXLogger =
            new Logger<>("cameraATargetCornersX_px", subdirString);
    private final Logger<Double[]> aTargetCornersYLogger =
            new Logger<>("cameraATargetCornersY_px", subdirString);

    private final Logger<Integer> aSetPipelineLogger =
            new Logger<>("cameraASetPipelineIndex", subdirString);

    private final Logger<Pose3d> estimatedPoseLogger =
            new Logger<>("estimatedPoseMeters", subdirString);

    private final Logger<PoseStrategy> poseStrategyLogger =
            new Logger<>("poseStrategy", subdirString);

    private Vision() {
        register();

        ArrayList<AprilTag> tags = new ArrayList<>(Constants.GLOBAL.TAG_LAYOUT.getTags());

        for (AprilTag tag : tags) {
            simTargets.add(
                    new SimVisionTarget(
                            tag.pose,
                            Constants.GLOBAL.APRILTAG_WIDTH_METERS,
                            Constants.GLOBAL.APRILTAG_HEIGHT_METERS,
                            tag.ID));
        }
    }

    public static Vision getInstance() {
        return instance;
    }

    public List<PhotonTrackedTarget> getCameraATargets() {
        return cameraA.getLatestResult().targets;
    }

    public double cameraAPipelineLatencySeconds() {
        return Units.millisecondsToSeconds(cameraA.getLatestResult().getLatencyMillis());
    }

    public int getCameraAPipelineIndex() {
        return cameraA.getPipelineIndex();
    }

    public void setCameraAPipelineIndex(int index) {
        cameraA.setPipelineIndex(index);
    }

    public PoseStrategy getPoseSolverStrategy() {
        return poseSolver.getStrategy();
    }

    public void setPoseSolverStrategy(PoseStrategy strategy) {
        poseSolver.setStrategy(strategy);
    }

    public Pair<Pose2d, Double> getCurrentPose2dEstimate() {
        if (estimatedPose == null || estimatedPose.getFirst() == null) return null;

        Pose2d asPose2d = estimatedPose.getFirst().toPose2d();
        double timestamp =
                Units.millisecondsToSeconds(System.currentTimeMillis())
                        - cameraAPipelineLatencySeconds();

        return Pair.of(asPose2d, timestamp);
    }

    @Override
    public void periodic() {
        poseSolver.setReferencePose(Localizer.getInstance().getEstimatedPose());

        var temp = poseSolver.update();

        if (temp.isPresent()) estimatedPose = temp.get();
        else estimatedPose = null;

        if (estimatedPose != null) {
            if (estimatedPose.getFirst() != null) {
                estimatedPoseLogger.update(estimatedPose.getFirst());
            }
        }

        ArrayList<Double> cornersX = new ArrayList<>();
        ArrayList<Double> cornersY = new ArrayList<>();

        for (PhotonTrackedTarget target : cameraA.getLatestResult().targets) {
            for (TargetCorner corner : target.getDetectedCorners()) {
                cornersX.add(corner.x);
                cornersY.add(corner.y);
            }
        }

        aTargetCornersXLogger.update(cornersX.toArray(new Double[] {}));
        aTargetCornersYLogger.update(cornersY.toArray(new Double[] {}));

        aLatencyLogger.update(
                Units.millisecondsToSeconds(cameraA.getLatestResult().getLatencyMillis()));

        aHasTargetsLogger.update(cameraA.getLatestResult().hasTargets());

        aSetPipelineLogger.update(cameraA.getPipelineIndex());

        poseStrategyLogger.update(poseSolver.getStrategy());
    }

    @Override
    public void simulationPeriodic() {
        simCameraA.processFrame(simPose);
    }
}
