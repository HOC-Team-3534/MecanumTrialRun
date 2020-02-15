package Autons;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.RobotMap;
import Autons.AutonCalculations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class AutonStateMachine0 extends AutonStateMachineBase implements AutonStateMachineInterface {

	int state = 1;
	int stateCnt = 0;

	double set_angle = Robot.drive.getAngle().getRadians();
	double last_angle_error = 0;

	WPI_TalonFX frontRight = RobotMap.frontRightMotor;
	WPI_TalonFX frontLeft = RobotMap.frontLeftMotor;

	AutonCalculations part1;
	double part1Heading = Math.PI / 6 * 9;
	double part1Rotation = 0;

	public AutonStateMachine0() {

	}

	@Override
	public void process() {

		int nextState = state;

		switch (state) {

		case 1:
		
			//any initialization code here
			nextState = 10;
			break;

		case 10:

			//calculate ramping and what not

			part1 = new AutonCalculations(3.0 * 1.537, RobotMap.maxVelocity, 0.75, 0.020);
			part1.calculate();

			nextState = 20;
			break;

		case 20:
			
			//drive
			double generalVelocity = part1.getVelocity();
			//this is where we want to change set angle if we want to rotate while driving
			double angle_error = set_angle - Robot.drive.getAngle().getRadians();
			double correctional_velocity = angle_error * 0.30 + (angle_error - last_angle_error) * 0;
			last_angle_error = angle_error;
			Robot.drive.drive(generalVelocity * Math.cos(part1Heading), generalVelocity * Math.sin(part1Heading), correctional_velocity, true);

			if(part1.isFinished()){
				nextState = 100;
			}
			break;

		case 100:

			RobotMap.frontLeftMotor.set(ControlMode.Velocity, 0);
			RobotMap.frontRightMotor.set(ControlMode.Velocity, 0);
			RobotMap.backLeftMotor.set(ControlMode.Velocity, 0);
			RobotMap.backRightMotor.set(ControlMode.Velocity, 0);

			break;
		}

		if (nextState != state) {
			state = nextState;
			stateCnt = 0;
		} else {
			stateCnt++;
		}

	}

}
