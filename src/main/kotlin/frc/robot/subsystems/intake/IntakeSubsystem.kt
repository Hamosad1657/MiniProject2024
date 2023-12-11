package frc.robot.subsystems.intake

import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Intake.MOTOR_ID)

	fun collect() = motor.set(Constants.MOTOR_OUTPUT)
	fun stop() = motor.stopMotor()

	val running: Boolean
		get() = motor.get() != 0.0

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Running", { running }, null)
	}
}