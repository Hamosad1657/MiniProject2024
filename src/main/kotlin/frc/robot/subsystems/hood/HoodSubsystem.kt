package frc.robot.subsystems.hood

import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.hood.HoodConstants.kD
import frc.robot.subsystems.hood.HoodConstants.kI
import frc.robot.subsystems.hood.HoodConstants.kP

object HoodSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID)
	private val encoder = CANCoder(RobotMap.Hood.ENCODER_ID)

	private val hoodController = PIDController(kP, kI, kD)

	private val reverseLimitSwitch = DigitalInput(RobotMap.Hood.LIMIT_CHANNEL)

	init {
		motor.forwardLimit = { hoodAngle.degrees >= HoodConstants.MAX_HOOD_ANGLE }
		motor.reverseLimit = { reverseLimitSwitch.get() }
	}

	fun getToAngleCommand(desiredAngleSupplier: () -> Rotation2d): Command {
		return RunCommand({
			val output = hoodController.calculate(hoodAngle.degrees, desiredAngleSupplier().degrees)
			motor.set(output)
		}, this).finallyDo { motor.set(0.0) }
	}


	val hoodAngle: Rotation2d
		get() = Rotation2d.fromDegrees(encoder.position * HoodConstants.GEAR_RATIO_ENCODER_TO_HOOD)
}