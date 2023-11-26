package frc.robot.subsystems.hood

import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object HoodSubsystem : SubsystemBase() {
	private val encoder = CANCoder(RobotMap.Hood.ENCODER_ID)
	private val hoodController = HoodConstants.PID_GAINS.toPIDController()
	private val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID)
	private val reverseLimitSwitch = DigitalInput(RobotMap.Hood.LIMIT_CHANNEL)

	private val hoodAngleDeg: Double
		get() = encoder.position * HoodConstants.GEAR_RATIO_ENCODER_TO_HOOD

	init {
		motor.forwardLimit = { hoodAngleDeg >= HoodConstants.MAX_HOOD_ANGLE }
		motor.reverseLimit = { reverseLimitSwitch.get() }
	}

	private fun getToAngle(desiredAngle: Double) {
		val output = hoodController.calculate(hoodAngleDeg, desiredAngle)
		return motor.set(output)
	}

	fun getToAngleCommand(desiredAngleSupplier: () -> Double): Command {
		return run {
			getToAngle(desiredAngleSupplier())
		}.finallyDo { motor.stopMotor() }
	}
}