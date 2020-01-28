package Autons;

public class AutonCalculations{

    public double total_distance = 0, total_time = 0, total_cycles = 0;
    private double max_cruise_velocity = 0, acceleration = 0, cycle_time = 0;
    private double acceleration_per_cycle = 0, cruise_velocity = 0;
    private double cycles_of_acceleration = 0, acceleration_distance = 0, acceleration_time = 0;
    private double cycles_of_cruising = 0, cruising_distance = 0, cruising_time = 0;

    private int current_cycle = 0;
    private double current_velocity = 0;

    public AutonCalculations(double distance, double velocity, double accel, double cycle){

        this.total_distance = distance;
        this.max_cruise_velocity = velocity;
        this.acceleration = accel;
        this.cycle_time = cycle;

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

    public double getVelocity(){

        current_cycle++;

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

        return current_velocity;

    }

    public boolean isFinished(){

        return current_velocity == 0 && current_cycle > total_cycles;

    }

}