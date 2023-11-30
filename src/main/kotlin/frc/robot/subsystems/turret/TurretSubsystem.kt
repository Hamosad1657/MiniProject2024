package frc.robot.subsystems.turret

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.turret.TurretConstants.CAMERA_MID_WIDTH
import frc.robot.subsystems.turret.TurretConstants.MAX_ANGLE_DEG
import frc.robot.subsystems.turret.TurretConstants.MIN_ANGLE_DEG
import frc.robot.subsystems.turret.TurretConstants.TOLERANCE_DEGREES
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs

object TurretSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Turret.MOTOR_ID)
	private val encoder = CANCoder(RobotMap.Turret.CANCODER_ID)

	/** CCW positive, according to standard mathematical conventions (and WPILib). */
	private val currentAngleDeg get() = encoder.position * TurretConstants.GEAR_RATIO_ENCODER_TO_TURRET
	private val farthestTurnAngle get() = if (currentAngleDeg >= 180) MIN_ANGLE_DEG else MAX_ANGLE_DEG

	init {
		encoder.configSensorDirection(false) // TODO: verify Turret CANCoder is CCW positive
		motor.inverted = false // TODO: verify Turret motor is CCW positive

		encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
		motor.forwardLimit = { currentAngleDeg >= MAX_ANGLE_DEG }
		motor.reverseLimit = { currentAngleDeg <= MIN_ANGLE_DEG }

		motor.config_kP(0, TurretConstants.kP)
		motor.config_kI(0, TurretConstants.kI)
		motor.config_kD(0, TurretConstants.kD)
	}

	private fun getToAngle(desiredAngle: Double) {
		motor.set(ControlMode.Position, MathUtil.inputModulus(desiredAngle, 0.0, 360.0))
	}

	fun getToAngleCommand(desiredAngle: Double): Command {
		return run {
			getToAngle(desiredAngle)
		}.until {
			val error = desiredAngle - currentAngleDeg
			abs(error) <= TOLERANCE_DEGREES
		}.finallyDo {
			motor.stopMotor()
		}
	}

	fun fullTurnCommand(): Command = getToAngleCommand(farthestTurnAngle)

	fun trackTargetCommand(trackedTargetSupplier: () -> PhotonTrackedTarget?): Command {
		var currentTurnAngle: Double? = null
		var lastTargetLeftCornerX: Double? = null

		return run {
			val target = trackedTargetSupplier()
			if (target == null) {
				if (currentTurnAngle == null) {
					currentTurnAngle =
						if (lastTargetLeftCornerX != null) guessTurnAngleByTargetCorner(lastTargetLeftCornerX!!)
						else farthestTurnAngle
				}

				getToAngle(currentTurnAngle!!)
			} else {
				lastTargetLeftCornerX = target.detectedCorners.minBy { it.x }.x
				currentTurnAngle = null

				val desiredAngle = currentAngleDeg - target.yaw
				getToAngle(desiredAngle)
			}
		}.finallyDo {
			motor.stopMotor()
		}
	}

	private fun guessTurnAngleByTargetCorner(cornerX: Double) =
		if (cornerX > CAMERA_MID_WIDTH) MAX_ANGLE_DEG else MIN_ANGLE_DEG

	fun closedLoopTeleopCommand(cwRotationSupplier: () -> Double, ccwRotationSupplier: () -> Double): Command {
		return run {
			val setpoint =
				(currentAngleDeg + cwRotationSupplier() - ccwRotationSupplier()) * TurretConstants.TELEOP_ANGLE_MULTIPLIER
			getToAngle(setpoint)
		}
	}
}
