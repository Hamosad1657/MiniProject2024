package frc.robot.subsystems.loader

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object LoaderSubsystem : SubsystemBase() {
	val motor = HaTalonFX(RobotMap.Loader.MOTOR_ID)

	fun load(ballsPerSecs: Double) {
		val setpoint = LoaderConstants.CONVEYOR_VELOCITY_CONVERSION * ballsPerSecs
		motor.set(ControlMode.Velocity, setpoint)
	}
}