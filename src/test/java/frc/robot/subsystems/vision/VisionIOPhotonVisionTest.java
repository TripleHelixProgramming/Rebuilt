package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonTrackedTarget;

class VisionIOPhotonVisionTest {

  private static PhotonTrackedTarget makeTarget(double ambiguity, Transform3d cameraToTarget) {
    var target = new PhotonTrackedTarget();
    target.fiducialId = 10;
    target.poseAmbiguity = ambiguity;
    target.bestCameraToTarget = cameraToTarget;
    return target;
  }

  private static final Transform3d THREE_METERS_AWAY =
      new Transform3d(3.0, 0.2, 0.5, new Rotation3d(0.0, 0.0, Math.PI));

  @Test
  @DisplayName("A solved single-tag target is kept")
  void solvedTargetIsKept() {
    assertTrue(VisionIOPhotonVision.hasPoseSolution(makeTarget(0.05, THREE_METERS_AWAY)));
  }

  @Test
  @DisplayName("A target with no 3D solve (identity transform, ambiguity -1) is dropped")
  void unsolvedTargetIsDropped() {
    // What the front-left camera sent for every tag at VAGLE1 Q6
    assertFalse(VisionIOPhotonVision.hasPoseSolution(makeTarget(-1.0, new Transform3d())));
  }

  @Test
  @DisplayName("A negative ambiguity alone is enough to drop the target")
  void negativeAmbiguityIsDropped() {
    assertFalse(VisionIOPhotonVision.hasPoseSolution(makeTarget(-1.0, THREE_METERS_AWAY)));
  }

  @Test
  @DisplayName("An identity transform alone is enough to drop the target")
  void identityTransformIsDropped() {
    assertFalse(VisionIOPhotonVision.hasPoseSolution(makeTarget(0.05, new Transform3d())));
  }

  @Test
  @DisplayName("Zero ambiguity (a perfectly unambiguous solve) is kept")
  void zeroAmbiguityIsKept() {
    assertTrue(VisionIOPhotonVision.hasPoseSolution(makeTarget(0.0, THREE_METERS_AWAY)));
  }
}
