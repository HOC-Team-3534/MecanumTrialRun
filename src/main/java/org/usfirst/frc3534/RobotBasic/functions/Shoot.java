package org.usfirst.frc3534.RobotBasic.functions;

import org.usfirst.frc3534.RobotBasic.Robot;
import org.usfirst.frc3534.RobotBasic.OI.Buttons;
import org.usfirst.frc3534.RobotBasic.RobotMap.FunctionStateDelay;
import org.usfirst.frc3534.RobotBasic.systems.Shooter.ShooterState;

public class Shoot extends FunctionBase implements FunctionInterface{

    long originalTime = 0;

    public Shoot(){

        reset();
        completed();

    }

    @Override
    public void process(){

        if(!running && Buttons.Shoot.getButton()){

            this.reset();

        }

        switch(this.state) {

        case 0:

            if(Buttons.Shoot.getButton()){
                this.started();
                this.state = 10;
                
            }

            break;

        case 10:

            originalTime = System.currentTimeMillis();
            Robot.shooter.setShooterState(ShooterState.SHOOT);
            this.state = 20;
            
            break;

        case 20:

            if(System.currentTimeMillis() - originalTime > 3000){

                this.state = 30;

            }

            break;

        case 30:

            Robot.shooter.setShooterState(ShooterState.STOP);
            this.state = 40;

            break;

        case 40:

            completed();

            break;
    
        }
    }
}