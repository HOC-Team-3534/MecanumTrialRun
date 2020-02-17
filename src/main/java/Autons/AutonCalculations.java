package Autons;

import org.usfirst.frc3534.RobotBasic.Robot;

public class AutonCalculations{

    public double total_distance = 0, total_time = 0, total_cycles = 0;
    private double max_cruise_velocity = 0, acceleration = 0, cycle_time = 0;
    private double acceleration_per_cycle = 0, cruise_velocity = 0;
    private double cycles_of_acceleration = 0, acceleration_distance = 0, acceleration_time = 0;
    private double cycles_of_cruising = 0, cruising_distance = 0, cruising_time = 0;

    private int current_cycle = 0;
    private double current_velocity = 0;
    public double currentXVelocity = 0.0;
    public double currentXYelocity = 0.0;
    public double finalMovementAngle = 0.0;
    public double finalX = 0.0;
    public double finalY = 0.0;
    public double currX = 0.0;
    public double currY = 0.0;
    public double nextX;
    public double nextY;

    public double sumErrorX, lastErrorX, sumErrorY, lastErrorY;
    public double kP = 0.25, kI = 0.0, kD = 2.5;

    public AutonCalculations(double finalX, double finalY, double velocity, double accel, double cycle){

        this.finalX = finalX;
        this.finalY = finalY;
        getCurrentPostion();
        this.total_distance = getTotalDistance();
        this.max_cruise_velocity = velocity;
        this.acceleration = accel;
        this.cycle_time = cycle;
        this.finalMovementAngle = getMovementAngle(finalX, finalY);
        this.nextX = currX;
        this.nextY = currY;

        reset();

    }

    public void reset(){

        current_cycle = -1;
        total_cycles = 3;

    }
    
    public void calculate(){

        acceleration_per_cycle = acceleration * cycle_time;
        cycles_of_acceleration = (max_cruise_velocity / acceleration / cycle_time) - 1;
        acceleration_distance = (cycles_of_acceleration / 2 * (cycles_of_acceleration + 1)) * acceleration_per_cycle * cycle_time;
        cruising_distance = total_distance - acceleration_distance * 2;
        cruise_velocity = max_cruise_velocity;
        cruising_time = cruising_distance / cruise_velocity;

        if(cruising_distance < 0){

            cycles_of_acceleration = Math.floor((1 + Math.sqrt(1 - 4 * -1 * (total_distance / acceleration_per_cycle / cycle_time))) / 2 * 1) - 1;
            acceleration_distance = (cycles_of_acceleration / 2 * (cycles_of_acceleration + 1)) * acceleration_per_cycle * cycle_time;
            cruising_distance = total_distance - acceleration_distance * 2;
            cruise_velocity = cycles_of_acceleration / acceleration_per_cycle;
            cruising_time = cruising_distance / cruise_velocity;

        }

        cycles_of_cruising = Math.round(cruising_time / cycle_time);
        cruising_distance = cycles_of_cruising * cruise_velocity * cycle_time;
        cruising_time = cruising_distance / cruise_velocity;
        acceleration_time = (cycles_of_acceleration + 1) * cycle_time;
        total_time = acceleration_time * 2 + cruising_time;
        total_distance = acceleration_distance * 2 + cruising_distance;
        total_cycles = (cycles_of_acceleration + 1) * 2 + cycles_of_cruising;

        System.out.println(total_cycles);

    }

    public void calcGeneralVelocity(){

        current_cycle++;

        getCurrentPostion();

        System.out.print(current_cycle);

        if(current_cycle == 0){

            current_velocity = 0.0;

        }else if(current_cycle <= cycles_of_acceleration){

            current_velocity += acceleration_per_cycle;

        }else if(current_cycle <= cycles_of_acceleration + cycles_of_cruising && cycles_of_cruising > 0){

        }else if(current_cycle <= cycles_of_acceleration * 2 + cycles_of_cruising){

            current_velocity -= acceleration_per_cycle;

        }else{

            System.out.println("The autonomous cycle has completed. You can stop calling the getVelocity() method");

        }

    }

    public double getTotalDistance(){

        return Math.sqrt(Math.pow(currY - finalY, 2) + Math.pow(currX - finalX, 2));

    }

    public void getCurrentPostion(){
        currX = Robot.drive.getCurrentX();
        currY = Robot.drive.getCurrentY();
    }

    public double getMovementAngle(double finalX, double finalY){

        double angle = Math.atan((finalY - currY) / (finalX - currX));
        double movementAngle = angle;
        if(finalX < currX && finalY > currY) { //quadrant 3

            movementAngle = Math.PI + movementAngle;

        } else if(finalX < currX && finalY < currY) { //quadrant 2

            movementAngle = Math.PI - movementAngle;

        } else if(finalX > currX && finalY < currY) { //quadrant 1
            
            

        } else if(finalX > currX && finalY > currY) {

            movementAngle = Math.PI * 2 - movementAngle;

        } else if(finalX == currX){
            
           if(finalY > currY){
               
               movementAngle = Math.PI * 3 / 2;
               
           }else{
               
               movementAngle = Math.PI / 2;
               
           }
            
        }else if(finalY == currY){
            
            if(finalX > currX){
               
               movementAngle = 0;
               
           }else{
               
               movementAngle = Math.PI;
               
           }
            
        }

        return movementAngle;

    }

    public double getXVelocity(boolean negated){

        double velocity = current_velocity * Math.cos(finalMovementAngle);
        double error = currX - nextX;
        int multipler = (negated) ? -1:1;
        nextX += multiplier * velocity * cycle_time;
        velocity += error * kP + sumErrorX * kI + (error - lastErrorX) * kD;
        return velocity;

    }

    public double getYVelocity(boolean negated){

        double velocity = current_velocity * Math.sin(finalMovementAngle);
        double error = currY - nextY;
        int multipler = (negated) ? -1:1;
        nextY += multiplier * velocity * cycle_time;
        velocity += error * kP + sumErrorY * kI + (error - lastErrorY) * kD;
        return velocity;

    }

    public boolean isFinished(){

        return current_velocity == 0 && current_cycle > total_cycles;

    }

}
