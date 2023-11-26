package frc.robot.subsystems.turret

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

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
		return run { getToAngle(desiredAngleSupplier) }.finallyDo { motor.set(0.0) }
	}

	fun getToAngle(desiredAngleSupplier: Rotation2d) {
		if (desiredAngleSupplier.invoke().degrees in 0.0..360.0) {
			val output = turretController.calculate(angle.degrees, desiredAngleSupplier().degrees)
			return motor.set(output)
		} else if (desiredAngleSupplier.invoke().degrees > 360.0) {
			val output = turretController
				.calculate((angle.degrees + 360) % 360, desiredAngleSupplier().degrees)
			return motor.set(output)
		}
	}

	val angle: Rotation2d
		get() = Rotation2d.fromDegrees(encoder.position * TurretConstants.gearRatioEncoderToTurret)
}