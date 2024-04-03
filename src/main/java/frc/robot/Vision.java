// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class Vision {

    ArrayList<Camera> cameras;

    public Vision() {
        cameras = new ArrayList<Camera>();
        cameras.add(new Camera(VisionConstants.FL_CAM_NAME, VisionConstants.flRobotToCam));
        cameras.add(new Camera(VisionConstants.FR_CAM_NAME, VisionConstants.frRobotToCam));
        cameras.add(new Camera(VisionConstants.BL_CAM_NAME, VisionConstants.blRobotToCam));
        cameras.add(new Camera(VisionConstants.BR_CAM_NAME, VisionConstants.brRobotToCam));
    }

    public List<Optional<EstimatedRobotPose>> getCameraEstimatedPoses() {
        ArrayList<Optional<EstimatedRobotPose>> retVal = new ArrayList<Optional<EstimatedRobotPose>>();
        for(Camera cam : cameras) {
            retVal.add(cam.getEstimatedGlobalPose());
        }
        return retVal;
    }

    public List<Optional<Matrix<N3, N1>>> getPoseStdDevs(List<Optional<EstimatedRobotPose>> poses) {
        ArrayList<Optional<Matrix<N3, N1>>> retVal = new ArrayList<Optional<Matrix<N3, N1>>>();
        int i = 0;
        for(Camera cam : cameras) {
            if(poses.get(i).isPresent()) {
                retVal.add(Optional.ofNullable(cam.getEstimationStdDevs(poses.get(i).get().estimatedPose.toPose2d())));
            }
            else {
                retVal.add(Optional.empty());
            }
            i++;
        }
        return retVal;
    }
}
