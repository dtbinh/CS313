package LineFollowing;

public class Values {
	
	//CONSTANTS
	public static final int EAST = 500;
	public static final int NORTH = 501;
	public static final int WEST = 502;
	public static final int SOUTH = 503;
	
	
	
	public static final int BLOCKED = 10;
	public static final int CLEAR = 20;
	public static final int VISITED = 30;
		
	//Threshold Values
	static int thLight = 500;
	static int thSonar = 120;
	
	//Grid Size
	static int GridX = 4;
	static int GridY = 4;
	
	//Initial Values
	public static int initFacing = EAST;
	
	//Target position
	static int targetX = 3;
	static int targetY = 3;

	//Robot position
	static int initRobotX = 0;
	static int initRobotY = 0;

	
}