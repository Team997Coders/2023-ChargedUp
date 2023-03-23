///**
//Copyright 2023 FRC Team 997
//
//This program is free software:
//you can redistribute it and/or modify it under the terms of the
//GNU General Public License as published by the Free Software Foundation,
//either version 3 of the License, or (at your option) any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//See the GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License along with this program.
//If not, see <https://www.gnu.org/licenses/>.
//*/
//package org.chsrobotics.competition2023.subsystems;
//
//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import java.util.ArrayList;
//import java.util.List;
//import org.chsrobotics.competition2023.Constants;
//import org.chsrobotics.competition2023.Constants.SUBSYSTEM.VISION;
//import org.chsrobotics.competition2023.Localizer;
//import org.chsrobotics.competition2023.Robot;
//import org.chsrobotics.lib.telemetry.HighLevelLogger;
//import org.chsrobotics.lib.telemetry.Logger;
//import org.chsrobotics.lib.util.Tuple2;
//import org.photonvision.EstimatedRobotPose;
//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.SimVisionSystem;
//import org.photonvision.targeting.PhotonTrackedTarget;
//import org.photonvision.targeting.TargetCorner;
//
//public class Vision implements Subsystem {
//    private static final Vision instance = new Vision();
//
//    private final PhotonCamera cameraA = new PhotonCamera(VISION.AT_CAMERA_FRONT_NAME);
//
//    private final PhotonCamera cameraB = new PhotonCamera(VISION.AT_CAMERA_BACK_NAME);
//
//    private final PhotonPoseEstimator camAEstimator;
//    private final PhotonPoseEstimator camBEstimator;
//
//    private EstimatedRobotPose frontCamEstimatedPose =
//            new EstimatedRobotPose(new Pose3d(), 0, List.of());
//    private EstimatedRobotPose backCamEstimatedPose =
//            new EstimatedRobotPose(new Pose3d(), 0, List.of());
//
//    private final SimVisionSystem simFrontCamera =
//            new SimVisionSystem(
//                    VISION.AT_CAMERA_FRONT_NAME,
//                    VISION.AT_CAMERAS_DIAG_FOV_DEGREES,
//                    VISION.ROBOT_TO_AT_FRONT_CAMERA,
//                    9970,
//                    VISION.AT_CAMERAS_HORIZONTAL_RESOLUTION_PX,
//                    VISION.AT_CAMERAS_VERTICAL_RESOLUTION_PX,
//                    VISION.AT_CAMERAS_SIMULATION_MIN_TARGET_AREA);
//
//    private final SimVisionSystem simBackCamera =
//            new SimVisionSystem(
//                    VISION.AT_CAMERA_BACK_NAME,
//                    VISION.AT_CAMERAS_DIAG_FOV_DEGREES,
//                    VISION.ROBOT_TO_AT_BACK_CAMERA,
//                    9970,
//                    VISION.AT_CAMERAS_HORIZONTAL_RESOLUTION_PX,
//                    VISION.AT_CAMERAS_VERTICAL_RESOLUTION_PX,
//                    VISION.AT_CAMERAS_SIMULATION_MIN_TARGET_AREA);
//
//    private Pose3d simPose = new Pose3d();
//
//    private final String subdirString = "vision";
//
//    private final Logger<Boolean> frontHasTargetsLogger =
//            new Logger<>("cameraFrontHasTargets", subdirString);
//
//    private final Logger<Boolean> backHasTargetsLogger =
//            new Logger<>("cameraBackHasTargets", subdirString);
//
//    private final Logger<Integer[]> frontTrackedTargetIDsLogger =
//            new Logger<>("cameraFrontTracketTargetIDs", subdirString);
//
//    private final Logger<Integer[]> backTrackedTargetIDsLogger =
//            new Logger<>("cameraBackTrackedTargetIDs", subdirString);
//
//    private final Logger<Double> frontLatencyLogger =
//            new Logger<>("cameraFrontPipelineLatency_s", subdirString);
//
//    private final Logger<Double> backLatencyLogger =
//            new Logger<>("cameraBackPipelineLatency_s", subdirString);
//
//    private final Logger<Double[]> frontTargetCornersXLogger =
//            new Logger<>("cameraFrontTargetCornersX_px", subdirString);
//    private final Logger<Double[]> frontTargetCornersYLogger =
//            new Logger<>("cameraFrontTargetCornersY_px", subdirString);
//
//    private final Logger<Double[]> backTargetCornersXLogger =
//            new Logger<>("cameraBackTargetCornersX_px", subdirString);
//    private final Logger<Double[]> backTargetCornersYLogger =
//            new Logger<>("cameraBackTargetCornersY_px", subdirString);
//
//    private final Logger<Integer> frontSetPipelineLogger =
//            new Logger<>("cameraFrontSetPipelineIndex", subdirString);
//
//    private final Logger<Integer> backSetPipelineLogger =
//            new Logger<>("cameraBackSetPipelineIndex", subdirString);
//
//    private final Logger<Double[]> camFrontEstimatedPoseLogger =
//            new Logger<>("camFrontEstimatedPose_m_m_rad", subdirString);
//
//    private final Logger<Double[]> camBackEstimatedPoseLogger =
//            new Logger<>("camBackEstimatedPose_m_m_rad", subdirString);
//
//    private Vision() {
//        register();
//
//        AprilTagFieldLayout layout;
//        try {
//            layout =
//                    AprilTagFieldLayout.loadFromResource(
//                            AprilTagFields.kDefaultField.m_resourceFile);
//        } catch (Exception e) {
//            layout = new AprilTagFieldLayout(List.of(), 1, 1);
//        }
//
//        simFrontCamera.addVisionTargets(layout);
//        simBackCamera.addVisionTargets(layout);
//
//        camAEstimator =
//                new PhotonPoseEstimator(
//                        layout,
//                        Constants.SUBSYSTEM.VISION.DEFAULT_POSE_STRATEGY,
//                        cameraA,
//                        Constants.SUBSYSTEM.VISION.ROBOT_TO_AT_FRONT_CAMERA);
//
//        camBEstimator =
//                new PhotonPoseEstimator(
//                        layout,
//                        Constants.SUBSYSTEM.VISION.DEFAULT_POSE_STRATEGY,
//                        cameraB,
//                        Constants.SUBSYSTEM.VISION.ROBOT_TO_AT_BACK_CAMERA);
//    }
//
//    public static Vision getInstance() {
//        return instance;
//    }
//
//    public double getCameraAPipelineLatencySeconds() {
//        return Units.millisecondsToSeconds(cameraA.getLatestResult().getLatencyMillis());
//    }
//
//    public double getCameraBPipelineLatencySeconds() {
//        return Units.millisecondsToSeconds(cameraB.getLatestResult().getLatencyMillis());
//    }
//
//    public int getCameraAPipelineIndex() {
//        return cameraA.getPipelineIndex();
//    }
//
//    public int getCameraBPipelineIndex() {
//        return cameraB.getPipelineIndex();
//    }
//
//    public void setCameraAPipelineIndex(int index) {
//        cameraA.setPipelineIndex(index);
//    }
//
//    public void setCameraBPipelineIndex(int index) {
//        cameraB.setPipelineIndex(index);
//    }
//
//    public Tuple2<EstimatedRobotPose> getCurrentPoseEstimates() {
//        return Tuple2.of(frontCamEstimatedPose, backCamEstimatedPose);
//    }
//
//    @Override
//    public void periodic() {
//        camAEstimator.setReferencePose(Localizer.getInstance().getEstimatedPose());
//        camBEstimator.setReferencePose(Localizer.getInstance().getEstimatedPose());
//
//        var camAEstimate = camAEstimator.update();
//        var camBEstimate = camBEstimator.update();
//
//        if (camAEstimate.isPresent()) {
//            frontCamEstimatedPose = camAEstimate.get();
//
//            camFrontEstimatedPoseLogger.update(
//                    new Double[] {
//                        frontCamEstimatedPose.estimatedPose.getX(),
//                        frontCamEstimatedPose.estimatedPose.getY(),
//                        frontCamEstimatedPose.estimatedPose.toPose2d().getRotation().getRadians()
//                    });
//        } else frontCamEstimatedPose = null;
//
//        if (camBEstimate.isPresent()) {
//            backCamEstimatedPose = camBEstimate.get();
//            camBackEstimatedPoseLogger.update(
//                    new Double[] {
//                        backCamEstimatedPose.estimatedPose.getX(),
//                        backCamEstimatedPose.estimatedPose.getY(),
//                        backCamEstimatedPose.estimatedPose.toPose2d().getRotation().getRadians()
//                    });
//        } else backCamEstimatedPose = null;
//
//        var aCornersX = new ArrayList<>();
//        var aCornersY = new ArrayList<>();
//        var bCornersX = new ArrayList<>();
//        var bCornersY = new ArrayList<>();
//
//        var aTargetIDs = new ArrayList<>();
//        var bTargetIDs = new ArrayList<>();
//
//        for (PhotonTrackedTarget target : cameraA.getLatestResult().targets) {
//            aTargetIDs.add(target.getFiducialId());
//            for (TargetCorner corner : target.getDetectedCorners()) {
//                aCornersX.add(corner.x);
//                aCornersY.add(corner.y);
//            }
//        }
//
//        for (PhotonTrackedTarget target : cameraB.getLatestResult().targets) {
//            bTargetIDs.add(target.getFiducialId());
//            for (TargetCorner corner : target.getDetectedCorners()) {
//                bCornersX.add(corner.x);
//                bCornersY.add(corner.y);
//            }
//        }
//
//        frontTargetCornersXLogger.update(aCornersX.toArray(new Double[] {}));
//        frontTargetCornersYLogger.update(aCornersY.toArray(new Double[] {}));
//        backTargetCornersXLogger.update(bCornersX.toArray(new Double[] {}));
//        backTargetCornersYLogger.update(bCornersY.toArray(new Double[] {}));
//
//        frontTrackedTargetIDsLogger.update(aTargetIDs.toArray(new Integer[] {}));
//        backTrackedTargetIDsLogger.update(bTargetIDs.toArray(new Integer[] {}));
//
//        frontLatencyLogger.update(getCameraAPipelineLatencySeconds());
//        backLatencyLogger.update(getCameraBPipelineLatencySeconds());
//
//        frontHasTargetsLogger.update(cameraA.getLatestResult().hasTargets());
//        backHasTargetsLogger.update(cameraB.getLatestResult().hasTargets());
//
//        frontSetPipelineLogger.update(cameraA.getPipelineIndex());
//        backSetPipelineLogger.update(cameraB.getPipelineIndex());
//    }
//
//    @Override
//    public void simulationPeriodic() {
//        simFrontCamera.processFrame(simPose);
//        simBackCamera.processFrame(simPose);
//    }
//
//    public void setSimState(Pose3d simPose) {
//        if (!Robot.isReal()) {
//            this.simPose = simPose;
//        } else {
//            HighLevelLogger.getInstance()
//                    .logWarning("Sim state should not be set on a real robot!");
//            HighLevelLogger.getInstance()
//                    .logWarning("There might be sim code still running somewhere!");
//        }
//    }
//}
