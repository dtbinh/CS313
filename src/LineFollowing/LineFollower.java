package LineFollowing;

import java.util.Iterator;
import java.util.LinkedList;

import compatibility.DifferentialPilot;
import compatibility.LCD;
import lejos.geom.Point;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import ch.aplu.robotsim.LightSensor;
import ch.aplu.robotsim.UltrasonicSensor;


class Robot {
	//================== Variables ==================	
	static private LightSensor L_Sens_Left;
	static private LightSensor L_Sens_Right;
	
	static private UltrasonicSensor ultraSonic;
  
	static DifferentialPilot pilot;
	//================= Constructor =================
	Robot( 
	  		LightSensor lS, LightSensor rS,
	  		UltrasonicSensor us, DifferentialPilot p) {
	  	
	 
		Map.init();
		Navigator.init();
		AStar.init();
		
	  	ultraSonic = us;
	  	
	  	L_Sens_Left = lS;
	  	L_Sens_Right = rS;
	  	
	  	pilot  = p;
	  	//pilot  = new DifferentialPilot(56, 177, Motor.B, Motor.C, false);  
	  	// parameters in inches;
	  	
	    // create each behavior
		Behavior junction = new Junction();
		Behavior move = new Move();
		
	    // define an array (vector) of existing behaviors, sorted by priority
		Behavior[] behaviors = { move, junction };
	
	    // add the behavior vector to a new arbitrator and start arbitration
		Arbitrator arbitrator = new Arbitrator(behaviors);
		
		LCD.drawString("Robot", 0, 0);
		//pilot.setTravelSpeed(100);  // cm per second
		//Button.waitForAnyPress();
		LCD.clear();
		
		drawSensor();
		
		forward();
		
		arbitrator.start();
  	}
	
	//==================== Robot Action ==================== 
	public static void stop() {
		/*==== Motor Implementation ====
		LeftMotor.stop();
		RightMotor.stop();
		 */
		
		//==== Pilot Implementation ====
		pilot.stop();
		
	}
	
	public static void rotate(int x) {
		pilot.rotate(x);
	}
	
	public static void rotateLeft() {
		pilot.rotateLeft();
		
	}

	public static void rotateRight() {
		pilot.rotateRight();
		
	}
	
	public static void forward() {
		/*==== Motor Implementation ====
		LeftMotor.forward();
		RightMotor.forward();
		*/
		
		pilot.forward();
	}
	
	public static int senseLeftLight() { 	
		return L_Sens_Left.getValue(); 
	}
	
	public static int senseRightLight() {
		return L_Sens_Right.getValue();
	}
	
	public static int senseSonar() { 
		System.out.println(ultraSonic.getDirection());
		return ultraSonic.getDistance(); 
		
	} 

	public static void drawSonic() {
		int distance = ultraSonic.getDistance();
		
		LCD.clear();
		LCD.drawString("ultraSonic: ", 0, 0);
		LCD.drawInt(distance, 0, 1);
		LCD.clear();
		
	}
	
	
	/**
	 * Draws values of the left and right
	 * Sensors on the screen. 
	 */
	public static void drawSensor() {

		int left = L_Sens_Left.getValue();
		int right = L_Sens_Right.getValue();
		
		LCD.drawString("Left Sensor: ", 0, 1);
		LCD.drawInt(left, 0, 2);
	
		LCD.drawString("Right Sensor: ", 0, 3);
		LCD.drawInt(right, 0, 4);
	}
}





//================= Navigation Layer =================
/**
 * @author Teo
 * Class Map wills tore information about 
 * the robots environment. It will be updated
 * in real time as the robot explores the maze
 */
class Map {
	
	static int virtualWorld[][];
	
	static int robotX;
	static int robotY;
	
	static int worldX;
	static int worldY;
	
	static int targetX;
	static int targetY;
	
	static int facing;
	
	public static void init() {
		robotX = Values.initRobotX;
		robotY = Values.initRobotY;
				
		facing = Values.initFacing;
		
		worldX = Values.GridX * 2 - 1;
		worldY = Values.GridY * 2 - 1;
		
		targetX = worldX - 1;
		targetY = worldY - 1;
		
		virtualWorld = new int[worldX][worldY];		
		
		int i;
		int j;
		
		for(i= 0; i < worldX; i++ ) {
			for(j = 0; j < worldY; j++)  {
				virtualWorld[i][j] = Values.BLOCKED;				
			}
		}
		
		for(i = 0; i < worldX; i+=2 ) {
			for(j = 0; j < worldY; j++)  {
				virtualWorld[i][j] = Values.CLEAR;
			}
		}
			
		for(i = 0; i < worldX; i++ ) {
			for(j = 0; j < worldY; j+= 2)  {
				virtualWorld[i][j] = Values.CLEAR;
			}
		}
		
		printMap();
	}
	
	public static int lookAhead() {
		int aheadX = 0;
		int aheadY = 0;
		
		switch(facing) {
		case Values.NORTH:
			aheadY = robotY + 1;
			aheadX = robotX;
			break;
			
		case Values.SOUTH:
			aheadY = robotY - 1;
			aheadX = robotX;
			break;
		
		case Values.EAST:
			aheadY = robotY;
			aheadX = robotX + 1;
			break;
		
		case Values.WEST:
			aheadY = robotY;
			aheadX = robotX - 1;
			break;
		}
		if(aheadX >= worldX || aheadX < 0 ||
				aheadY >= worldY || aheadY < 0) {
			return Values.BLOCKED;
		} else {
			return virtualWorld[aheadX][aheadY];
		}
	}

	public static Point getNextNodeCoord() {
		Point p = new Point(0, 0);
		
		switch(facing) {
		case Values.NORTH:
			p.y = robotY + 2;
			p.x = robotX;
			break;
			
		case Values.SOUTH:
			p.y = robotY - 2;
			p.x = robotX;
			break;
		
		case Values.EAST:
			p.y = robotY;
			p.x = robotX + 2;
			break;
		
		case Values.WEST:
			p.y = robotY;
			p.x = robotX - 2;
			break;
		}		
		return p;
	}
	
	public static void printMap() {
		int i;
		int j;
		for(i= worldX - 1; i >=0; i-- ) {
			for(j= 0; j < worldY; j++ ) {
				if(j == robotX && i == robotY)  {
					System.out.print(" [R] ");
				} else if(j == targetX && i == targetY) {
						System.out.print(" [X] ");
				} else if(virtualWorld[j][i] == Values.CLEAR){
					System.out.print(" [ ] ");
				} else if(virtualWorld[j][i] == Values.BLOCKED) {
					System.out.print(" ... ");
				} else if(virtualWorld[j][i] == Values.VISITED) {
					System.out.print(" [#] ");
				} 
			}
			
			System.out.println();
		}
	}

	public static boolean atTarget() {
		if(robotX == targetX && robotY == targetY) {
			return true;
		}
		return false;
	}
	
	public static void updateMap(int x) {
		int aheadX = 0;
		int aheadY = 0;
		
		switch(facing) {
		case Values.NORTH:
			aheadY = robotY + 1;
			aheadX = robotX;
			break;
			
		case Values.SOUTH:
			aheadY = robotY - 1;
			aheadX = robotX;
			break;
		
		case Values.EAST:
			aheadY = robotY;
			aheadX = robotX + 1;
			break;
		
		case Values.WEST:
			aheadY = robotY;
			aheadX = robotX - 1;
			break;
		}
		if(aheadX >= worldX || aheadX < 0 ||
				aheadY >= worldY || aheadY < 0) {
			return;
		} else if(virtualWorld[aheadX][aheadY] != Values.VISITED){
			virtualWorld[aheadX][aheadY] = x;
		}
	}
	
	public static void updateRobotPosition() {
		
		virtualWorld[robotX][robotY] = Values.VISITED; 
		
		switch(facing) {
			case Values.NORTH:
				robotY++;
				break;
				
			case Values.SOUTH:
				robotY--;
				break;
			
			case Values.EAST:
				robotX++;
				break;
			
			case Values.WEST:
				robotX--;
				break;
		}
		
		virtualWorld[robotX][robotY] = Values.VISITED;
		
		LCD.drawString("RobotX = ", 0, 4);
		LCD.drawInt(robotX, 0, 5);
		
		LCD.drawString("RobotY", 0, 6);
		LCD.drawInt(robotX, 0, 7);
	}
}


class Node {
	
	LinkedList<Integer> path = new LinkedList<Integer>();
	
	public int way;
	
	private int cost;
	
	public void setCost(int v) 					{ cost = v; 	}
	public void setPath(LinkedList<Integer> p) 	{ path = p; 	}
	
	public int 					getCost() 		{ return cost; 	}
	public LinkedList<Integer> 	getPath() 		{ return path;	}
	
	public void updatePath(int w) {
		//path.add(w);
		path.addLast(w);
	}
	
	Node(int c) {
		cost = c;
	}
	
	Node(int c, LinkedList<Integer> p) {
		cost = c;
		path = p;
	}
	
	Node(int c, int w) {
		cost = c;
		way = w;
	}
}

class Navigator {

	//================= Initialisation =================
	public static void init() {
		
		
	}
	
	
	//================= Work =================
	public static void adjustLeft()
	{
		Robot.stop();
		Robot.rotateLeft();
		
		while(Robot.senseRightLight() < Values.thLight && !Move.stop);
		
		Robot.stop();
		Robot.forward();
	}
	
	public static void adjustRight()
	{
		Robot.stop();
		Robot.rotateRight();
		
		while(Robot.senseRightLight() < Values.thLight && !Move.stop);
		
		Robot.stop();
		Robot.forward();
	}
	
	public static void advance() {
		Robot.forward();
		Map.updateRobotPosition();
	}
	
	public static void robotFace(int f) {
		switch(Map.facing) {
		
		case Values.NORTH:
			helperFace(Values.NORTH,Values.SOUTH, Values.WEST, Values.EAST, f );
			break;
			
		case Values.SOUTH:
			helperFace(Values.SOUTH,Values.NORTH, Values.EAST, Values.WEST, f );
			break;
		
		case Values.EAST:
			helperFace(Values.EAST,Values.WEST, Values.NORTH, Values.SOUTH, f );
			break;
		
		case Values.WEST:
			helperFace(Values.WEST,Values.EAST, Values.SOUTH, Values.NORTH, f );
			break;
		
		}
	}
	
	public static void helperFace(final int ahead, final int back, final int left, final int right, int change) {
		if(change == back ) {
			Robot.rotate(-180);
			Map.facing = back;
		} else if (change == left) {
			Robot.rotate(-90);
			Map.facing = left;
		} else if(change == right) {
			Robot.rotate(90);
			Map.facing = right;
		}
	}
	
	public static void moveForward() {
		
		while(!(Robot.senseLeftLight() < Values.thLight && Robot.senseRightLight() < Values.thLight)) {
			if(			//Offcourse Right
				Robot.senseLeftLight() < Values.thLight) 
			{
				Robot.drawSensor();
				adjustRight();
			} else if(	//Offcourse Left
					Robot.senseRightLight() < Values.thLight) 
			{
				Robot.drawSensor();
				adjustLeft();
			}
		}
		Robot.stop();
		
	}

}


class AStar {
	static Node[][] openNodes;
	static Node[][] closedNodes;	
	
	public static final int NODE_DISTANCE = 2;
	
	public static void init() {
		int i = Values.targetX * 2 + 1;
		int j = Values.targetY * 2 + 1;
		
		
		openNodes = new Node[i][j];
		closedNodes = new Node[i][j];
		
		openNodes[0][0] = new Node(0);
	}
	
	public static int currentF() {
		return NODE_DISTANCE + openNodes[Map.robotX][Map.robotY].getCost();
	}
	
	public static int predictedF(int x, int y) {
		return ((Values.targetX - x) + (Values.targetY - y)); 	
	
	}
	
	public static int calculateTotalCost(int x, int y) {
		return predictedF(x, y) + currentF();
	}
	
	public static int[] scanNeightbours() 
	{
		int[] clearPaths = new int[4];
		int index = 0;
		
		
		for(int i = Values.EAST; i <= Values.SOUTH; i++) {
			Navigator.robotFace(i);
			
			if(Map.lookAhead() != Values.BLOCKED ) {
			
				if(Robot.senseSonar() == -1) {
				
					Map.updateMap(Values.CLEAR);
					clearPaths[index++] = i;
					
					System.out.println(Robot.senseSonar() + " Path Clear");
				} else if(Robot.senseSonar() < Values.thSonar) {
					Map.updateMap(Values.BLOCKED);
					System.out.println(Robot.senseSonar() + "Path Blocked");
				}
			}
			
			try{Thread.sleep(1000);}catch(Exception e){}
		}
		return clearPaths;
	}

	public static void addOpenNode(int x, int y, int c, int w) {		
		openNodes[x][y] = new Node(c, w);
	}
	
	public static void removeOpenNode(int x, int y) {
		openNodes[x][y] = null;
	}
	
	public static void addClosedNode(int x, int y) {
		closedNodes[x][y] = openNodes[x][y];
	}

	public static Node getOpenNode(int x, int y) {
		return openNodes[x][y];
	}

	public static Node getNextNode() {
		
		Node n = null;
		int minCost = 10000;
		
		for(int i = 0; i < Values.targetX * 2 + 1; i++) {
			for(int j = 0; j < Values.targetY * 2 + 1; j++) {
				if(openNodes[i][j] != null && openNodes[i][j].getCost() < minCost ){
					minCost = openNodes[i][j].getCost();
					n = openNodes[i][j];
				}
			}
		}
		
		closedNodes[Map.robotX][Map.robotY].updatePath(n.way);
		
		
		return n;
	}

	public static void goBack() {
		if(Map.robotX == 0 && Map.robotY == 0) {
			return;
		} else {
			LinkedList path = closedNodes[Map.robotX][Map.robotY].getPath();
			
			Iterator i = path.iterator();
			
			 while(i.hasNext()) {
		         int dir = (int) i.next();
		         
		         switch(dir) {
		         case Values.NORTH:
		        	 dir = Values.SOUTH;
		        	 break;
		         case Values.SOUTH:
		        	 dir = Values.NORTH;
		        	 break;
		         case Values.EAST:
		        	 dir = Values.WEST;
		        	 break;
		         case Values.WEST:
		        	 dir = Values.EAST;
		        	 break;
		         }
		         
		         Navigator.robotFace(dir);
		     }
		}
	}

	public static void goTo(Node n) {
		Navigator.robotFace((int) closedNodes[Map.robotX][Map.robotY].getPath().getLast());
	}
	
	public static void updateNeightbourNodes(int[] clearPaths) {
		//update their cost value
		for(int i = 0; i < clearPaths.length; i++) {
			Navigator.robotFace(clearPaths[i]);
			int x = Map.robotX;
			int y = Map.robotY;
			switch(clearPaths[i]) {
			case Values.NORTH:
				y += NODE_DISTANCE;
				break;
			case Values.SOUTH:
				y -= NODE_DISTANCE;
				break;
			case Values.EAST:
				x += NODE_DISTANCE;
				break;
			case Values.WEST:
				x -= NODE_DISTANCE;
				break;
			}
			Node n = getOpenNode(x, y);
			if(n == null) {
				addOpenNode(x, y, calculateTotalCost(x, y), clearPaths[i]);
			} else {
				int cost = calculateTotalCost(x, y);
				if(n.getCost() > cost) {
					n.setCost(cost);
					//n.setPath(Navigator.getOpenNode(Map.robotX, Map.robotY).getPath());
				}
			}
		}
	}
	
	
}


//================= Action Layer =================
class Junction implements Behavior
{
	boolean stop = false;	
	
	public void action()			// what to do
	{	
		stop = false;
		Robot.pilot.travel(250);
		
		Map.printMap();
		
		if(Map.atTarget() == true) {
			Robot.stop();
			return;
		}
		
		LCD.drawString("Junction behaviour", 0, 0);
		Robot.drawSensor();
		
		
		
		//===== Important Stuff here =====
			
			//scan neighbour nodes
			int clearPaths[] = AStar.scanNeightbours();
			
			//update their cost value
			AStar.updateNeightbourNodes(clearPaths);
			
			//close current node
			AStar.addClosedNode(Map.robotX, Map.robotY);
			AStar.removeOpenNode(Map.robotX, Map.robotY);
			
			
			//getNodeWithSmallestCost
			Node n = AStar.getNextNode();
		
			//go to next node
			AStar.goBack();
			AStar.goTo(n);
			
		
		//================================
		
			
			
		try{Thread.sleep(1000);}catch(Exception e){}
		LCD.clear();
		
		Robot.stop();
		
		Navigator.advance();
		Navigator.advance();
		
		stop = true;
	}

	public void suppress()			// how to stop doing it
	{	
		Robot.stop();
		stop = true;
	}

	public boolean takeControl()	// when to start doing it
	{		
		return (Robot.senseLeftLight() < Values.thLight &&
				Robot.senseRightLight() < Values.thLight);
	}
}


class Move implements Behavior
{
	static boolean stop = false;
	
	public void action()			// what to do
	{	
		stop = false;
		
		LCD.drawString("Move behaviour", 0, 0);
		Robot.drawSensor();
		
		Navigator.moveForward();
		
		try{Thread.sleep(100);}catch(Exception e){}
		LCD.clear();
	
	}
	
	public void suppress()			// how to stop doing it
	{	
		Robot.stop();
		stop = true;
	}

	public boolean takeControl()	// when to start doing it
	{
		return (Robot.senseLeftLight() > Values.thLight &&
				Robot.senseRightLight() > Values.thLight);
	}	
}
