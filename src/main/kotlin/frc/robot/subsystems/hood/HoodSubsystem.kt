package frc.robot.subsystems.hood

import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object HoodSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID)
	private val encoder = CANCoder(RobotMap.Hood.ENCODER_ID)


	private val reverseLimitSwitch = DigitalInput(RobotMap.Hood.LIMIT_CHANNEL)

	init {
		motor.forwardLimit = { hoodAngle.degrees >= HoodConstants.MAX_HOOD_ANGLE }

		motor.reverseLimit = { reverseLimitSwitch.get() }

	}

	val hoodAngle: Rotation2d
		get() = Rotation2d.fromDegrees(encoder.position * HoodConstants.GEAR_RATIO_ENCODER_TO_HOOD)

}