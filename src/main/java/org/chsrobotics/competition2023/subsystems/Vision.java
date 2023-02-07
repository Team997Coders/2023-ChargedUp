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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import org.chsrobotics.competition2023.Constants;
import org.chsrobotics.competition2023.Constants.SUBSYSTEM.VISION;
import org.chsrobotics.competition2023.Localizer;
import org.chsrobotics.competition2023.Robot;
import org.chsrobotics.lib.telemetry.HighLevelLogger;
import org.chsrobotics.lib.telemetry.Logger;
import org.chsrobotics.lib.util.Tuple2;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision implements Subsystem {
    private static final Vision instance = new Vision();

    private final PhotonCamera cameraA = new PhotonCamera(VISION.AT_CAMERA_A_NAME);

    private final PhotonCamera cameraB = new PhotonCamera(VISION.AT_CAMERA_B_NAME);

    private final PhotonPoseEstimator camAEstimator;
    private final PhotonPoseEstimator camBEstimator;

    private EstimatedRobotPose camAEstimatedPose = new EstimatedRobotPose(new Pose3d(), 0);
    private EstimatedRobotPose camBEstimatedPose = new EstimatedRobotPose(new Pose3d(), 0);

    private final SimVisionSystem simCameraA =
            new SimVisionSystem(
                    VISION.AT_CAMERA_A_NAME,
                    VISION.AT_CAMERAS_DIAG_FOV_DEGREES,
                    VISION.ROBOT_TO_AT_CAMERA_A,
                    9970,
                    VISION.AT_CAMERAS_HORIZONTAL_RESOLUTION_PX,
                    VISION.AT_CAMERAS_VERTICAL_RESOLUTION_PX,
                    VISION.AT_CAMERAS_SIMULATION_MIN_TARGET_AREA);

    private final SimVisionSystem simCameraB =
            new SimVisionSystem(
                    VISION.AT_CAMERA_B_NAME,
                    VISION.AT_CAMERAS_DIAG_FOV_DEGREES,
                    VISION.ROBOT_TO_AT_CAMERA_B,
                    9970,
                    VISION.AT_CAMERAS_HORIZONTAL_RESOLUTION_PX,
                    VISION.AT_CAMERAS_VERTICAL_RESOLUTION_PX,
                    VISION.AT_CAMERAS_SIMULATION_MIN_TARGET_AREA);

    private Pose3d simPose = new Pose3d();

    private final String subdirString = "vision";

    private final Logger<Boolean> aHasTargetsLogger =
            new Logger<>("cameraAHasTargets", subdirString);

    private final Logger<Boolean> bHasTargetsLogger =
            new Logger<>("cameraBHasTargets", subdirString);

    private final Logger<Integer[]> aTrackedTargetIDsLogger =
            new Logger<>("cameraATracketTargetIDs", subdirString);

    private final Logger<Integer[]> bTrackedTargetIDsLogger =
            new Logger<>("cameraBTrackedTargetIDs", subdirString);

    private final Logger<Double> aLatencyLogger =
            new Logger<>("cameraAPipelineLatency_s", subdirString);

    private final Logger<Double> bLatencyLogger =
            new Logger<>("cameraBPipelineLatency_s", subdirString);

    private final Logger<Double[]> aTargetCornersXLogger =
            new Logger<>("cameraATargetCornersX_px", subdirString);
    private final Logger<Double[]> aTargetCornersYLogger =
            new Logger<>("cameraATargetCornersY_px", subdirString);

    private final Logger<Double[]> bTargetCornersXLogger =
            new Logger<>("cameraBTargetCornersX_px", subdirString);
    private final Logger<Double[]> bTargetCornersYLogger =
            new Logger<>("cameraBTargetCornersY_px", subdirString);

    private final Logger<Integer> aSetPipelineLogger =
            new Logger<>("cameraASetPipelineIndex", subdirString);

    private final Logger<Integer> bSetPipelineLogger =
            new Logger<>("cameraBSetPipelineIndex", subdirString);

    private final Logger<Double[]> camAEstimatedPoseLogger =
            new Logger<>("camAEstimatedPose_m_m_rad", subdirString);

    private final Logger<Double[]> camBEstimatedPoseLogger =
            new Logger<>("camBEstimatedPose_m_m_rad", subdirString);

    private Vision() {
        register();

        AprilTagFieldLayout layout;
        try {
            layout =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.kDefaultField.m_resourceFile);
        } catch (Exception e) {
            layout = new AprilTagFieldLayout(List.of(), 1, 1);
        }

        simCameraA.addVisionTargets(layout);
        simCameraB.addVisionTargets(layout);

        camAEstimator =
                new PhotonPoseEstimator(
                        layout,
                        Constants.SUBSYSTEM.VISION.DEFAULT_POSE_STRATEGY,
                        cameraA,
                        Constants.SUBSYSTEM.VISION.ROBOT_TO_AT_CAMERA_A);

        camBEstimator =
                new PhotonPoseEstimator(
                        layout,
                        Constants.SUBSYSTEM.VISION.DEFAULT_POSE_STRATEGY,
                        cameraB,
                        Constants.SUBSYSTEM.VISION.ROBOT_TO_AT_CAMERA_B);
    }

    public static Vision getInstance() {
        return instance;
    }

    public double getCameraAPipelineLatencySeconds() {
        return Units.millisecondsToSeconds(cameraA.getLatestResult().getLatencyMillis());
    }

    public double getCameraBPipelineLatencySeconds() {
        return Units.millisecondsToSeconds(cameraB.getLatestResult().getLatencyMillis());
    }

    public int getCameraAPipelineIndex() {
        return cameraA.getPipelineIndex();
    }

    public int getCameraBPipelineIndex() {
        return cameraB.getPipelineIndex();
    }

    public void setCameraAPipelineIndex(int index) {
        cameraA.setPipelineIndex(index);
    }

    public void setCameraBPipelineIndex(int index) {
        cameraB.setPipelineIndex(index);
    }

    public Tuple2<EstimatedRobotPose> getCurrentPoseEstimates() {
        return Tuple2.of(camAEstimatedPose, camBEstimatedPose);
    }

    @Override
    public void periodic() {
        camAEstimator.setReferencePose(Localizer.getInstance().getEstimatedPose());
        camBEstimator.setReferencePose(Localizer.getInstance().getEstimatedPose());

        var camAEstimate = camAEstimator.update();
        var camBEstimate = camBEstimator.update();

        if (camAEstimate.isPresent()) {
            camAEstimatedPose = camAEstimate.get();

            camAEstimatedPoseLogger.update(
                    new Double[] {
                        camAEstimatedPose.estimatedPose.getX(),
                        camAEstimatedPose.estimatedPose.getY(),
                        camAEstimatedPose.estimatedPose.toPose2d().getRotation().getRadians()
                    });
        } else camAEstimatedPose = null;

        if (camBEstimate.isPresent()) {
            camBEstimatedPose = camBEstimate.get();
            camBEstimatedPoseLogger.update(
                    new Double[] {
                        camBEstimatedPose.estimatedPose.getX(),
                        camBEstimatedPose.estimatedPose.getY(),
                        camBEstimatedPose.estimatedPose.toPose2d().getRotation().getRadians()
                    });
        } else camBEstimatedPose = null;

        var aCornersX = new ArrayList<>();
        var aCornersY = new ArrayList<>();
        var bCornersX = new ArrayList<>();
        var bCornersY = new ArrayList<>();

        var aTargetIDs = new ArrayList<>();
        var bTargetIDs = new ArrayList<>();

        for (PhotonTrackedTarget target : cameraA.getLatestResult().targets) {
            aTargetIDs.add(target.getFiducialId());
            for (TargetCorner corner : target.getDetectedCorners()) {
                aCornersX.add(corner.x);
                aCornersY.add(corner.y);
            }
        }

        for (PhotonTrackedTarget target : cameraB.getLatestResult().targets) {
            bTargetIDs.add(target.getFiducialId());
            for (TargetCorner corner : target.getDetectedCorners()) {
                bCornersX.add(corner.x);
                bCornersY.add(corner.y);
            }
        }

        aTargetCornersXLogger.update(aCornersX.toArray(new Double[] {}));
        aTargetCornersYLogger.update(aCornersY.toArray(new Double[] {}));
        bTargetCornersXLogger.update(bCornersX.toArray(new Double[] {}));
        bTargetCornersYLogger.update(bCornersY.toArray(new Double[] {}));

        aTrackedTargetIDsLogger.update(aTargetIDs.toArray(new Integer[] {}));
        bTrackedTargetIDsLogger.update(bTargetIDs.toArray(new Integer[] {}));

        aLatencyLogger.update(getCameraAPipelineLatencySeconds());
        bLatencyLogger.update(getCameraBPipelineLatencySeconds());

        aHasTargetsLogger.update(cameraA.getLatestResult().hasTargets());
        bHasTargetsLogger.update(cameraB.getLatestResult().hasTargets());

        aSetPipelineLogger.update(cameraA.getPipelineIndex());
        bSetPipelineLogger.update(cameraB.getPipelineIndex());
    }

    @Override
    public void simulationPeriodic() {
        simCameraA.processFrame(simPose);
        simCameraB.processFrame(simPose);
    }

    public void setSimState(Pose3d simPose) {
        if (!Robot.isReal()) {
            this.simPose = simPose;
        } else {
            HighLevelLogger.getInstance()
                    .logWarning("Sim state should not be set on a real robot!");
            HighLevelLogger.getInstance()
                    .logWarning("There might be sim code still running somewhere!");
        }
    }
}
