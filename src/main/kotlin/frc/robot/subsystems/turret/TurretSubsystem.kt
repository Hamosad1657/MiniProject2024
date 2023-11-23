package frc.robot.subsystems.turret

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.turret.TurretConstants.kD
import frc.robot.subsystems.turret.TurretConstants.kI
import frc.robot.subsystems.turret.TurretConstants.kP

object TurretSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Turret.HATALONFX_ID)
	private val encoder = CANCoder(RobotMap.Turret.CANCODER_ID)

	private val turretController = PIDController(kP, kI, kD)

	init {
		encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
		motor.forwardLimit = { angle.degrees >= TurretConstants.MAX_ANGLE }
		motor.reverseLimit = { angle.degrees <= TurretConstants.MIN_ANGLE }

		motor.config_kP(0, TurretConstants.kP)
		motor.config_kI(0, TurretConstants.kI)
		motor.config_kD(0, TurretConstants.kD)
	}

	fun getToAngleCommand(desiredAngleSupplier: () -> Rotation2d): Command {
		return RunCommand({
			val output = turretController.calculate(angle.degrees, desiredAngleSupplier().degrees)
			motor.set(output)
		}).finallyDo { motor.set(0.0) }
	}

	val angle: Rotation2d
		get() = Rotation2d.fromDegrees(encoder.position * TurretConstants.gearRatioEncoderToTurret)
}