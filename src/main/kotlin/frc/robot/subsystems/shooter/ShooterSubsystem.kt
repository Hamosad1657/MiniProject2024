package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ShooterSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Shooter.HATALONFX_ID)

	init {
		motor.config_kP(0, ShooterConstants.kP)
		motor.config_kI(0, ShooterConstants.kI)
		motor.config_kD(0, ShooterConstants.kD)
	}

	fun shootBallsCommand(angularVelocity: AngularVelocity): Command {
		return RunCommand({ motor.set(ControlMode.Velocity, angularVelocity.degPs) }, this)
			.finallyDo { motor.set(0.0) }
	}
}