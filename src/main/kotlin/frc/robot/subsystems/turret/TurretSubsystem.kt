package frc.robot.subsystems.turret

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object TurretSubsystem : SubsystemBase() {
	private val turretMotor = HaTalonFX(RobotMap.Turret.HATALONFX_ID)
	private val canCoder = CANCoder(RobotMap.Turret.CANCODER_ID)
}