package frc.robot.utilities;
/**
 * Code to calculate maximum height, horizontal range and time of flight for a projectile
 * Author  : 
 * Version : 0.3
 * Bugs    : Unknown

 * Sample input:
u=5
a=30

or

a=30
u=5

 * here u is projection velocity and a is angle of projection. u is taken in metres per second and a is taken in degrees.
 * Please don't put space between u or a, equal and value. Don't forget to see the changelog at the end.
 * If you find any bugs, please feel free to comment. Enjoy!
*******************************************/

import java.io.*;

public class ProjectileMotion
{
    static double velocity, angle, range1, time, maxh1, range2, range3, maxh2;
    public static void main(String[] args) throws IOException{
        Input();
        Projectile p1=new Projectile(velocity,angle);
        range1=p1.MaxHorRange();
        time=p1.TimeOfFlight();
        maxh1=p1.MaxHeight();
        range2=p1.RusingHandu();
        range3=p1.Rusingtandu();
        maxh2=p1.HusingT();
        System.out.println("Horizontal range for this projectile is : " + range1 + " metres");
        System.out.println("Maximum height for this projectile is : " + maxh1 + " metres");
        System.out.println("Time of flight for this projectile is : " + time + " seconds");
        System.out.println("Horizontal range for this projectile calculated using only velocity and maximum height is " + range2 + " metres");
        System.out.println("Horizontal range for this projectile calculated using only time of flight and velocity is " + range3 + " metres");
        System.out.println("Maximum height for this projectile calculated using only time of flight is " + maxh2 + " metres");
    }
    public static void Input() throws IOException{
        String helper="";
        BufferedReader k=new BufferedReader(new InputStreamReader(System.in));
        String[] s=new String[2];
        System.out.println("Enter angle of projection in degrees and projection velocity in metres per second: ");
        for(int c1=0;c1<2;c1++){
            s[c1]=k.readLine();
            if(s[c1].charAt(0) == 'u'){
                for(int c2=2; c2<s[c1].length(); c2++)
                    helper+=s[c1].charAt(c2);
    
                velocity = Double.parseDouble(helper);
                helper="";
            }
            else if(s[c1].charAt(0) == 'a'){
                for(int c3=2; c3<s[c1].length(); c3++)
                    helper+=s[c1].charAt(c3);
    
                angle = Double.parseDouble(helper);
                helper="";
                if(Math.abs(angle)>90){
                    System.out.print("Invalid angle");
                    System.exit(0);
                }
            }
            else{
                System.out.print("Invalid input.");
                System.exit(0);
            }
        }
    }
}

