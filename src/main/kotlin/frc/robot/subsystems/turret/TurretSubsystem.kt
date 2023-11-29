package frc.robot.subsystems.turret

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs
import kotlin.math.absoluteValue

object TurretSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Turret.MOTOR_ID)
	private val encoder = CANCoder(RobotMap.Turret.CANCODER_ID)


	init {
		encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
		motor.forwardLimit = { angle.degrees >= TurretConstants.MAX_ANGLE }
		motor.reverseLimit = { angle.degrees <= TurretConstants.MIN_ANGLE }

		motor.config_kP(0, TurretConstants.kP)
		motor.config_kI(0, TurretConstants.kI)
		motor.config_kD(0, TurretConstants.kD)
	}

	fun getToAngleCommand(desiredAngleSupplier: () -> Rotation2d): Command {
		return run { getToAngle(desiredAngleSupplier()) }.finallyDo { motor.set(0.0) }
	}

	private fun getToAngle(desiredAngle: Rotation2d) {
		motor.set(ControlMode.Position, MathUtil.inputModulus(desiredAngle.degrees, 0.0, 360.0))
	}


	val angle: Rotation2d
		get() = Rotation2d.fromDegrees(encoder.position * TurretConstants.GEAR_RATIO_ENCODER_TO_TURRET)


	// Command that takes a supplier of PhotonTrackedTarget and causes the turret to rotate towards the target.
	// If there is no target, the turret simply stops moving.

	fun turnTowardsTargetCommand(targetSupplier: () -> PhotonTrackedTarget?): Command {
	return run {
		val target = targetSupplier()
		if(target != null) {getToAngle(target.yaw.degrees)}
		}.finallyDo { motor.set(0.0) }
	}

	fun getToAngleAndEndCommand(desiredAngleSupplier: () -> Rotation2d, tolerance: Rotation2d): Command {
		return getToAngleCommand { desiredAngleSupplier() }.until {
			val error = desiredAngleSupplier().degrees - angle.degrees
			abs(error) >= tolerance.degrees
		}
	}

	fun fullTurnCommand(): Command {
		var setpoint = 0.0
		if (angle.degrees >= 180.0) {
			setpoint = 0.0
		}
		else {
			setpoint = 360.0
		}
		return run {
			getToAngleAndEndCommand({setpoint.degrees}, TurretConstants.TOLERANCE.degrees)
		}
	}

}
