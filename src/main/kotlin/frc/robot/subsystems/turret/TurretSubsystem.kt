package frc.robot.subsystems.turret

import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
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

	private val cwLimit = DigitalInput(RobotMap.Turret.CW_LIMIT_SWITCH_CHANNEL)
	private val ccwLimit = DigitalInput(RobotMap.Turret.CCW_LIMIT_SWITCH_CHANNEL)

	private val turretController = PIDController(kP, kI, kD)

	init {
		encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
		motor.forwardLimit = { cwLimit.get() }
		motor.reverseLimit = { ccwLimit.get() }
	}

	fun getToAngleCommand(desiredAngleSupplier: () -> Rotation2d): Command {
		val output = turretController.calculate(getAngle().degrees, desiredAngleSupplier().degrees)
		return RunCommand({motor.set(output)}).finallyDo { motor.set(0.0) }
	}

	fun getAngle(): Rotation2d {
		return Rotation2d(encoder.position * TurretConstants.gearRatioEncoderToTurret)
	}
}