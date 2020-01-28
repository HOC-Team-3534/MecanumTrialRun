package org.usfirst.frc3534.RobotBasic.systems;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class PIDArm extends SystemBase implements SystemInterface {

	private WPI_TalonSRX arm;

	private final int MOVE = 0;
	private final int STABILIZE = 1;
	private final int STOP = 2;
	
	private final double maxArmVelocity = 8;
	private final double armLength = 25;
	
	private boolean moveFirstTime = true, stabilizeFirstTime = true;
	
	private PIDEncoderProcessor pidArmEP;

	public PIDArm() {

		pidArmEP.configurePIDVA(0.8, 0.0, 1 / maxArmVelocity);
		pidArmEP.configureEncoder(arm.getSensorCollection().getQuadraturePosition(), RobotMap.ticksPerMotorRotation, armLength * 2, .020);

	}

	@Override
	public void process() {
		
		double axisInput = Robot.oi.getController1().getY(Hand.kLeft);
		double armPower = 0.0;

		if (Math.abs(axisInput) >= .05) {


		} else {


		}

		/*
		switch () {

		case MOVE:
			
			if(moveFirstTime) {
				
				pidArmEP.configureEncoder(arm.getSensorCollection().getQuadraturePosition(), RobotMap.countsPerRevEncoders, 50, .020);
				moveFirstTime = false;
				stabilizeFirstTime = true;
				
			}
			
			if(Math.abs(axisInput) > .8) {
				
				armPower = pidArmEP.calculate(arm.getSensorCollection().getQuadraturePosition(), (Math.abs(axisInput) / axisInput) * maxArmVelocity);
				
			}else {
				
				armPower = pidArmEP.calculate(arm.getSensorCollection().getQuadraturePosition(), (Math.abs(axisInput) / axisInput) * maxArmVelocity * .5);
				
			}

			break;
		case STABILIZE:
			
			if(stabilizeFirstTime) {
				
				pidArmEP.configureEncoder(arm.getSensorCollection().getQuadraturePosition(), RobotMap.countsPerRevEncoders, 50, .020);
				moveFirstTime = true;
				stabilizeFirstTime = false;
				
			}

			armPower = pidArmEP.calculate(arm.getSensorCollection().getQuadraturePosition(), 0.0);
			
			break;
		case STOP:

			break;

		}*/
		
		arm.set(armPower);

	}

}
