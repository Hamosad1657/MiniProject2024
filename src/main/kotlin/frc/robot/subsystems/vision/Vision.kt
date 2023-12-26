package frc.robot.subsystems.vision
//
//import com.hamosad1657.lib.math.filters.HaDebouncer
//import edu.wpi.first.apriltag.AprilTag
//import edu.wpi.first.apriltag.AprilTagFieldLayout
//import edu.wpi.first.math.geometry.*
//import edu.wpi.first.wpilibj2.command.Subsystem
//import frc.robot.subsystems.turret.TurretConstants
//import org.photonvision.*
//import org.photonvision.targeting.PhotonTrackedTarget
//
//// THIS IS A PLACEHOLDER.
//
//object Vision : Subsystem {
//
//	val currentBestTag: PhotonTrackedTarget? = null
//
//	val aprilTags: AprilTagFieldLayout =
//		AprilTagFieldLayout(
//			mutableListOf(
//				AprilTag(
//					0,
//					Pose3d(
//						0.0, 0.0, 0.0,
//						Rotation3d(0.0, 0.0, 0.0)
//					)
//				),
//				AprilTag(
//					0,
//					Pose3d(
//						0.0, 0.0, 0.0,
//						Rotation3d(0.0, 0.0, 0.0)
//					)
//				)
//			), 0.0, 0.0
//		)
//
//	private val robotToCamera = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0))
//
//	private val camera = PhotonCamera("")
//
//	private val poseEstimator =
//		PhotonPoseEstimator(
//			aprilTags,
//			PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
//			camera,
//			robotToCamera,
//		).apply {
//			setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
//		}
//
//	/**
//	 * Gets the estimated robot position from the PhotonVision camera.
//	 * Returns null if it doesn't detect any April Tags.
//	 */
//	var estimatedGlobalPose: EstimatedRobotPose? = poseEstimator.update().orElse(null)
//
//
//	// From here downwards is written by Shaked
//
//	var latestPipelineResult = camera.latestResult // Updated in periodic
//		private set
//
//
//	// TODO: Find a better way to do this
//
//	/**
//	 * Every possible tag is in the list, with the tag ID corresponding to the index.
//	 * The value at the index is true/false depending on whether the tag was continuously
//	 * detected for over [TurretConstants.TAG_DETECTION_TIME_SEC]. If it was detected for
//	 * a shorter amount of time it is treated as a false positive and not counted.
//	 */
//	val tagsDetected
//		// This is done so that you can still access the information in
//		// tagsDetected_mutable outside of object Vision, but you can't modify it.
//		get() = tagsDetected_mutable.toBooleanArray()
//
//	/**
//	 * Every possible tag is in the list, with the tag ID corresponding to the index.
//	 * The value at the index is true/false depending on whether the tag was continuously
//	 * detected for over [TurretConstants.TAG_DETECTION_TIME_SEC]. If it was detected for
//	 * a shorter amount of time it is treated as a false positive and not counted.
//	 */
//	private val tagsDetected_mutable: ArrayList<Boolean> = ArrayList()
//
//	/**
//	 * There is one debouncer for every possible tag, with the tad ID corresponding
//	 * to the index in the list.
//	 */
//	private val tagDetectionTracker: ArrayList<HaDebouncer> = ArrayList();
//
//	init {
//		for (tag in aprilTags.tags) {
//			tagsDetected_mutable.add(false)
//			tagDetectionTracker.add(HaDebouncer(TurretConstants.TAG_DETECTION_TIME_SEC))
//		}
//	}
//
//	fun isSpecificTagDetected(tagID: Int): Boolean {
//		for (tag in latestPipelineResult.targets) {
//			if (tag.fiducialId == tagID) return true
//		}
//		return false
//	}
//
//	val isAnyTagDetected: Boolean
//		get() {
//			for (tagDetected in tagsDetected) {
//				if (tagDetected) return true
//			}
//			return false
//		}
//
//	override fun periodic() {
//		latestPipelineResult = camera.latestResult
//
//		for (i in 0..aprilTags.tags.size) {
//			tagsDetected_mutable[i] = tagDetectionTracker[i].debounceRisingEdge(isSpecificTagDetected(i));
//		}
//	}
//}

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.*
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import kotlin.math.PI

object Vision : SubsystemBase() {

	private val robotToCamera = Transform3d(Translation3d(), Rotation3d(0.0, 0.0, PI / 2))
	private val camera = PhotonCamera("AprilTag-Cam")

	var aprilTags: AprilTagFieldLayout =
		AprilTagFieldLayout(
			listOf(
				AprilTag(
					5,
					Pose3d(
						0.0, 0.0, 0.0,
						Rotation3d(0.0, 0.0, 0.0)
					)
				)
			), 10.0, 10.0
		)

	private val poseEstimator =
		PhotonPoseEstimator(
			aprilTags,
			PoseStrategy.MULTI_TAG_PNP,
			camera,
			robotToCamera,
		).apply {
			setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
		}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 * Returns null if it doesn't detect any April Tags.
	 */
	var estimatedGlobalPose: EstimatedRobotPose? = poseEstimator.update().orElse(null)

	val latestResult get() = camera.latestResult

	val bestTag get() = latestResult.bestTarget

	fun calibrateTagPositions(transform: Transform3d) {
		aprilTags = AprilTagFieldLayout(
			latestResult.targets.map { target ->
				AprilTag(
					target.fiducialId,
					(target.bestCameraToTarget + transform).let { Pose3d(it.translation, it.rotation) }
				)
			}, 10.0, 10.0
		)
	}

	fun getTag(tagID: Int) = latestResult.getTargets().getOrNull(tagID)

	override fun periodic() {
		poseEstimator.update().orElse(null)
	}
}
