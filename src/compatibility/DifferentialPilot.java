package compatibility;

import ch.aplu.robotsim.Gear;

public class DifferentialPilot {
	Gear gear;
	
	public DifferentialPilot(Gear g) {
		gear = g;
	}

	public void setSpeed(int speed) {	gear.setSpeed(speed);	}
    public void forward() 			{	gear.forward();			}
    public void backward()			{	gear.backward(); 		}
    public void stop() 				{	gear.stop();			}
    
    public void travel(int i) { gear.forward(i); }
    
    public void rotateLeft() { 		gear.left();	}
    public void rotateRight() {		gear.right();	}
    public void rotate(int angle) { 
    	if(angle < 0) {
    		gear.left(-angle / 90 * 575);
    	} else {
    		gear.right(angle / 90 * 575);
    	}
    }
}

class RegulatedMotor {}