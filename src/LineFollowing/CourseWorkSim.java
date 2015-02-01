package LineFollowing;

import java.awt.Color;

import lejos.nxt.Button;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import ch.aplu.jgamegrid.GGMouse;
import ch.aplu.jgamegrid.GGMouseListener;
import ch.aplu.jgamegrid.GameGrid;
import ch.aplu.jgamegrid.Location;
import ch.aplu.robotsim.*;
import ch.aplu.util.Monitor;


class LCD 
{
	static void drawString(String s, int x, int y) {	
		System.out.print(s);
	}
	static void clear() {
		System.out.println();
	}	
	static void drawInt(int v, int x, int y) {
		System.out.print(v);
	}
}


public class CourseWorkSim
{
	public static void main(String[] args)
	{
		try {
			new Sim();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}


class Sim {
	LegoRobot robot;
	
	static Motor LeftMotor;
	static Motor RightMotor;
	
	static LightSensor L_Sens_Left;
	static LightSensor L_Sens_Right;
	
    static TouchSensor ts1;
    static TouchSensor ts2;
	

    
    private static int xOld, yOld;
    
    // ================= Environment =================
	static
	{
		RobotContext.showNavigationBar();
		  
		RobotContext.useObstacle("sprites/hazelnut.gif", 200, 250);
	
		//RobotContext.setLocation(10, 10);
		RobotContext.setStartDirection(0);
		RobotContext.setStartPosition(130, 370);
		//RobotContext.useObstacle(RobotContext.channel);
		RobotContext.useBackground("sprites/grid.gif");
	}
	
	private static void _init(final GameGrid gg)
	{
	    gg.setTitle("Drag with left mouse button to draw the track");
	    gg.addMouseListener(
	      new GGMouseListener()
	      {
	        public boolean mouseEvent(GGMouse mouse)
	        {
	          Location loc =
	            gg.toLocationInGrid(mouse.getX(), mouse.getY());
	          switch (mouse.getEvent())
	          {
	            case GGMouse.lPress:
	              xOld = loc.x;
	              yOld = loc.y;
	              break;
	            case GGMouse.lDrag:
	              gg.getBg().drawLine(xOld, yOld, loc.x, loc.y);
	              xOld = loc.x;
	              yOld = loc.y;
	              break;
	            case GGMouse.lRelease:
	              Monitor.wakeUp();  // Start simulation
	              break;
	          }
	          return true;
	        }
	      }, GGMouse.lPress | GGMouse.lDrag | GGMouse.lRelease);
	    gg.getBg().setPaintColor(Color.black);
	    gg.getBg().setLineWidth(4);
	  }
	
	// ================= Setup =================
	public Sim() throws Exception
	{
		Setup();
		new LineFollower(LeftMotor, RightMotor, L_Sens_Left, L_Sens_Right);
	
	}
    
	public void Setup() {
		robot = new LegoRobot();
		
		RightMotor = new Motor(MotorPort.A);
		LeftMotor = new Motor(MotorPort.B);
		    
		L_Sens_Left = new LightSensor(SensorPort.S1);
		L_Sens_Right = new LightSensor(SensorPort.S2);
		
		ts1 = new TouchSensor(SensorPort.S3); // right sensor
		ts2 = new TouchSensor(SensorPort.S4); // left sensor
		
	    robot.addPart(L_Sens_Left);
	    robot.addPart(L_Sens_Right);
	    
	    robot.addPart(RightMotor);
	    robot.addPart(LeftMotor);
	   
	}
	


}

class LineFollower {
	//================== Variables ==================
    static int th = 40;
	
	static Motor LeftMotor;
	static Motor RightMotor;
	
	static LightSensor L_Sens_Left;
	static LightSensor L_Sens_Right;
	
    static TouchSensor ts1;
    static TouchSensor ts2;
    
    
    //================= Constructor =================
    LineFollower(Motor lM, Motor rM, 
    		LightSensor lS, LightSensor rS) {
    	
    	LeftMotor = lM;
    	RightMotor = rM;
    	
    	L_Sens_Left = lS;
    	L_Sens_Right = rS;
    	
    	try {
			Run();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    }
	
    
	//==================== Work ==================== 
	public static void Run() throws Exception
	{
		/*
        // create each behavior
		Behavior junction = new Junction();
		Behavior move = new Move();
	
        // define an array (vector) of existing behaviors, sorted by priority
		Behavior behaviors[] = { move, junction };

        // add the behavior vector to a new arbitrator and start arbitration
		Arbitrator arbitrator = new Arbitrator(behaviors);
		
		arbitrator.start();
		*/
		
		
		
		while(true)
		{
			if(			//Going straight
				L_Sens_Left.getValue() > th &&
				L_Sens_Right.getValue() > th) 
			{	
				GoStraight();
			} else if(	//Junction
				L_Sens_Left.getValue() < th &&
				L_Sens_Right.getValue() < th) 
			{	
				drawSensor();
				Junction();
			} else if(	//Offcourse Right
					L_Sens_Left.getValue() < th) 
			{
				drawSensor();
				GoRight();
			} else if(	//Offcourse Left
					L_Sens_Right.getValue() < th) 
			{
				drawSensor();
				GoLeft();
			}	
			//try{Thread.sleep(100);} catch(Exception e) {}
		}
	}
	
	public static void GoStraight() {
		LeftMotor.forward();
		RightMotor.forward();
	}
	
	public static void GoLeft()
	{
		RightMotor.forward();
		LeftMotor.stop();
		
		while(L_Sens_Right.getValue() < th);
						
		RightMotor.stop();
			
	}
	
	public static void GoRight()
	{
		LeftMotor.forward();
		RightMotor.stop();
		
		while(L_Sens_Left.getValue() < th);
						
		LeftMotor.stop();
	}
	
	public static void Junction() {
		LeftMotor.stop();
		RightMotor.stop();
	}
	
	
	public static void drawSensor() {
		int left = L_Sens_Left.getValue();
		int right = L_Sens_Right.getValue();
	
		LCD.clear();
		LCD.clear();
		LCD.drawString("Left Sensor: ", 0, 0);
		LCD.drawInt(left, 0, 1);
		
		LCD.drawString(" | ", 0, 0);
		
		LCD.drawString("Right Sensor: ", 0, 0);
		LCD.drawInt(right, 0, 1);	
	}
}


class Junction implements Behavior
{
	// state variables for this behavior
	boolean stop = false;
	
	// what to do
	// Mandatory class (declared in Behaviour interface)
	public void action()
	{	
		stop = false;
		try
		{
			while (!stop)
			{
			
			}
		}
		catch(Exception ex)
		{
		}
	}

	// how to stop doing it
	// Mandatory class (declared in Behaviour interface)
	public void suppress()
	{	
		stop = true;
		BehaviorRobot.stop();
	}

	// when to start doing it
	// Mandatory class (declared in Behaviour interface)
	public boolean takeControl()
	{
		return true;
	}	
}


class Move implements Behavior
{
	// state variables for this behavior
	boolean stop = false;
	
	// what to do
	// Mandatory class (declared in Behaviour interface)
	public void action()
	{	
		stop = false;
		try
		{
			while (!stop)
			{
			
			}
		}
		catch(Exception ex)
		{
		}
	}

	// how to stop doing it
	// Mandatory class (declared in Behaviour interface)
	public void suppress()
	{	
		stop = true;
		BehaviorRobot.stop();
	}

	// when to start doing it
	// Mandatory class (declared in Behaviour interface)
	public boolean takeControl()
	{
		return true;
	}	
}



