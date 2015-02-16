package compatibility;

import LineFollowing.Robot;
import ch.aplu.jgamegrid.Actor;
import ch.aplu.robotsim.Gear;

public class DifferentialPilot {
	Gear gear;
	
	public DifferentialPilot(Gear g) {
		gear = g;
	}

	public void setSpeed(int speed) {	gear.setSpeed(speed);	}
	public int getSpeed() 			{	return gear.getSpeed();	}
    public void forward() 			{	gear.forward();			}
    public void backward()			{	gear.backward(); 		}
    public void stop() 				{	gear.stop();			}
    
    public void travel(int i) { gear.forward(i); }
    
    public void rotateLeft() { 		gear.left();	}
    public void rotateRight() {		gear.right();	}
    public void rotate(int angle) { 
    	Actor r = Robot.robot.getRobot();
		r.turn(angle);
    }
}

class RegulatedMotor {}