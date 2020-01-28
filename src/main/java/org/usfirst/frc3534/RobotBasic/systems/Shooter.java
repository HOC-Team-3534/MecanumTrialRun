package org.usfirst.frc3534.RobotBasic.systems;

import org.usfirst.frc3534.RobotBasic.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Shooter extends SystemBase implements SystemInterface {

	private WPI_TalonFX shooter = RobotMap.shooter;

    ShooterState shooterState = ShooterState.STOP;

    public Shooter(){

    }

    @Override
    public void process(){

        switch(shooterState){
        case SHOOT:

            setShooterPower(shooterState.value); 

            break;

        case STOP:

            setShooterPower(shooterState.value); 

            break;

        }

    }

    public enum ShooterState{
        
        SHOOT(RobotMap.PowerOutput.shooter_shooter_shoot.power),
        STOP(RobotMap.PowerOutput.shooter_shooter_stop.power);

        double value;

        private ShooterState(double value){

            this.value = value;

        }

    }

    public void setShooterState(ShooterState state){

        shooterState = state;

    }

    private void setShooterPower(double power){

        shooter.set(ControlMode.Velocity, power);

    }
}