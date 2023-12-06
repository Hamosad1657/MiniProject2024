package frc.robot.subsystems.intake

import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object IntakeSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Intake.MOTOR_ID)

	fun stopIntake() {
		motor.stopMotor()
	}

	fun collect() {
		motor.set(IntakeConstants.MOTOR_OUTPUT)
	}
}