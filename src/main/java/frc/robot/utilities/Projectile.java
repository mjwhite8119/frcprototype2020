package frc.robot.utilities;

public class Projectile{
    double u,T,R,t,H;
    final double g = 9.80665;

    public Projectile(double u, double T){
        this.u = u;
        this.T = Math.abs((3.1416/180)*T);
    }

    public void set(){
        // Horizonal range of projectile
        this.R = u*u*Math.sin(2*T)/g;
        
        // Total time of flight
        this.t = 2*u*Math.sin(T)/g;

        // Maximum height of projectile
        this.H = u*u*Math.pow(Math.sin(T),2)/(2*g);
    }

    // Return max horizonal range
    public double MaxHorRange(){
        set();
        return this.R;
    }

    // Return total time of flight
    public double TimeOfFlight(){
        set();
        return this.t;
    }

    // Return max height of parabola
    public double MaxHeight(){
        set();
        return this.H;
    }

    public double RusingHandu(){
        set();
        return 4*Math.sqrt(H*(u*u/(2*g)-H));
    }
    public double HusingT(){
        set();
        return g*t*t*0.125;
    }
    public double Rusingtandu(){
        set();
        return 0.5*t*Math.sqrt(4*u*u-g*t*g*t);
    }

    // Return horizonal distance at a specific time
    public double HorDistanceAtTime(double time){
        return u*Math.cos(T)*time;
    }

    // Return height at specific time
    public double HeightAtTime(double time){
        return (u*Math.sin(T)*time-0.5*g*time*time);
    }

    // Return height at a specific distance
    public double HeightAtDistance(double x){
        return (x*Math.tan(T)-(g*x*x/(2*Math.pow(u*Math.cos(T),2))));
    }
}