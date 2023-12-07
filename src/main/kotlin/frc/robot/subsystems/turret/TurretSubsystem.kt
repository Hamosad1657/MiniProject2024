package frc.robot.subsystems.turret

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import org.photonvision.targeting.PhotonTrackedTarget
import frc.robot.subsystems.turret.TurretConstants as Constants

object TurretSubsystem : SubsystemBase() {
	val motor = HaTalonFX(RobotMap.Turret.MOTOR_ID).apply {
		inverted = false // TODO: verify Turret motor is CCW positive
		forwardLimit = { currentAngleDeg >= Constants.MAX_ANGLE_DEG }
		reverseLimit = { currentAngleDeg <= Constants.MIN_ANGLE_DEG }

		config_kP(0, Constants.kP)
		config_kI(0, Constants.kI)
		config_kD(0, Constants.kD)
	}

	private val encoder = CANCoder(RobotMap.Turret.CANCODER_ID).apply {
		configSensorDirection(false) // TODO: verify Turret CANCoder is CCW positive
		configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)

	}

	var setpoint = 0.degrees
		private set

	/** CCW positive, according to standard mathematical conventions (and WPILib). */
	val currentAngleDeg get() = encoder.position * Constants.GEAR_RATIO_ENCODER_TO_TURRET
	val farthestTurnAngle get() = if (currentAngleDeg >= 180) Constants.MIN_ANGLE_DEG else Constants.MAX_ANGLE_DEG

	private val tagDetectionDebouncer = Debouncer(Constants.TAG_DETECTION_TIME_SEC)

	init {
		setAngleSetpoint(Constants.MIN_ANGLE_DEG.degrees)
	}

	/**
	 * @param desiredAngle Can be any value, is not required to be in 0 to 360
	 */
	fun setAngleSetpoint(desiredAngle: Rotation2d) {
		setpoint = desiredAngle
	}

	fun guessTurnAngleByTargetCorner(cornerX: Double) =
		if (cornerX > Constants.CAMERA_MID_WIDTH) Constants.MAX_ANGLE_DEG else Constants.MIN_ANGLE_DEG

	/**
	 * Return if saw tag of specified ID for over [Constants.TAG_DETECTION_TIME_SEC] seconds.
	 */
	fun seeingTag(tagID: Int, trackedTarget: PhotonTrackedTarget?): Boolean {
		var seeingTheTagNow = false

		if (trackedTarget != null) {
			if (trackedTarget.fiducialId == tagID) {
				seeingTheTagNow = true
			}
		}
		return tagDetectionDebouncer.calculate(seeingTheTagNow)
	}
}
