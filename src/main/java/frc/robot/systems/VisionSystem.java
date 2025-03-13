package frc.robot.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.AprilTag;
import frc.robot.constants.SimConstants;
import frc.robot.constants.VisionConstants;

import java.util.ArrayList;
import org.photonvision.PhotonCamera;
/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionSystem {
	private final PhotonCamera reefCamera;
	private final PhotonCamera stationCamera;

	/**
	* Creates a new RaspberryPiSim.
	*/
	public VisionSystem() {
		reefCamera = new PhotonCamera("reef_at_cam");
		stationCamera = new PhotonCamera("station_at_cam");
	}

	/**
	 * Prints all raw apriltag data to console.
	 */
	public void printRawData() {
		for (AprilTag tag: getAprilTags()) {
			System.out.println("AprilTag " + tag.getTagID() + " -> " + tag.getPose().toString());
		}
	}

	/**
	 * Returns a list of all april tags from reef and station camera.
	 * @return all april tags
	 */
	public ArrayList<AprilTag> getAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();
		atList.addAll(getReefAprilTags());
		atList.addAll(getStationAprilTags());
		return atList;
	}

	/**
	 * Returns the AprilTag with the specified ID.
	 * @param id the ID of the AprilTag to retrieve
	 * @return the AprilTag with the specified ID, or null if not found
	 */
	public AprilTag getAprilTagWithID(int id) {
		return getAprilTags()
			.stream()
			.filter(tag -> tag.getTagID() == id)
			.findFirst()
			.orElse(null);
	}

	/**
	 * Returns a list of all april tags from reef CV camera.
	 * @return all visible reef april tags.
	 */
	public ArrayList<AprilTag> getReefAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();

		var results = reefCamera.getLatestResult();
		if (results.hasTargets()) {
			for (var target: results.getTargets()) {
				AprilTag at = new AprilTag(
					target.getFiducialId(),
					reefCamera.getName(),
					new Translation3d(), //camera vector, unused
					new Translation3d(
						target.getBestCameraToTarget().getX(),
						target.getBestCameraToTarget().getY(),
						target.getBestCameraToTarget().getZ()
					),
					new Rotation3d(
						target.getBestCameraToTarget().getRotation().getX(),
						target.getBestCameraToTarget().getRotation().getY(),
						target.getBestCameraToTarget().getRotation().getZ()
					)
				);
				if (at.getPose().getTranslation().getNorm()
					< VisionConstants.MAX_TAG_TARGET_DISTANCE_X
					&& target.getPoseAmbiguity() <= VisionConstants.MAX_TAG_AMBIGUITY) {
					atList.add(at);
				}
			}
		}
		return atList;
	}

	/**
	 * Returns all april tags visible from Station CV Camera.
	 * @return list of all april tags
	 */
	public ArrayList<AprilTag> getStationAprilTags() {
		ArrayList<AprilTag> atList = new ArrayList<>();

		var results = stationCamera.getLatestResult();
		if (results.hasTargets()) {
			for (var target: results.getTargets()) {
				AprilTag at = new AprilTag(
					target.getFiducialId(),
					stationCamera.getName(),
					new Translation3d(), //camera vector, unused
					new Translation3d(
						-target.getBestCameraToTarget().getX(),
						-target.getBestCameraToTarget().getY(),
						-target.getBestCameraToTarget().getZ()
					),
					new Rotation3d(
						target.getBestCameraToTarget().getRotation().getX(),
						target.getBestCameraToTarget().getRotation().getY(),
						target.getBestCameraToTarget().getRotation().getZ()
					)
				);
				if (at.getPose().getTranslation().getNorm() < SimConstants.CAM_DISTANCE_READ
					&& target.getPoseAmbiguity() <= VisionConstants.MAX_TAG_AMBIGUITY) {
					atList.add(at);
				}
			}
		}
		return atList;
	}

	/**
	 * Updates the raspberry pi's values given the current robot pose.
	 * Not used for teleop functionality.
	 * @param pose
	 */
	public void update(Pose2d pose) {
		// pass
	}
}
