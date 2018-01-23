import java.util.ArrayList;
import java.awt.Color;
import java.util.Date;
import javax.swing.*;
import java.util.StringTokenizer;
import java.io.*;

public class World
{
    public static enum HEADING { NORTH, SOUTH, EAST, WEST, NORTHEAST, NORTHWEST, SOUTHEAST, SOUTHWEST };
    
    private static JFrame field;
    public static double xLength, yLength, zLength;
    public static double windSpeed;
    public static HEADING windHeading;
    public static int NUM_DRONES;
    public static double T;
    public static int NUM_SIMULATION_CYCLES;
    private static BufferedReader in;
    private static Drone[] drones;
    public static int numTargets;
    public static double[] targetXPos, targetYPos, targetZPos, targetRadii;
    private static int turn;
    private static final String[] files = {"../data.txt", 
                                           "../firstTrial.txt",
                                           "../collisionDetection.txt",
                                           "../circleTurn.txt",
                                           "../presentation.txt"};
    private static final int fileIndex = 0;
    
    public World()
    {
        StringTokenizer input;
        try {
            in = new BufferedReader(new FileReader(files[fileIndex]));
            input = new StringTokenizer(in.readLine());
        } catch (IOException e) {
            System.out.println("File error - " + e.getMessage());
            System.exit(0);
            in = null;
            input = new StringTokenizer("");
        }
        this.xLength = Double.parseDouble(input.nextToken());
        this.yLength = Double.parseDouble(input.nextToken());
        this.zLength = Double.parseDouble(input.nextToken());
        this.windSpeed = Double.parseDouble(input.nextToken());
        this.windHeading = HEADING.values()[Integer.parseInt(input.nextToken())];
        this.NUM_DRONES = Integer.parseInt(input.nextToken());
        this.T = Double.parseDouble(input.nextToken());
        this.NUM_SIMULATION_CYCLES = Integer.parseInt(input.nextToken());
        this.drones = new Drone[NUM_DRONES];
        this.turn = 0;
        this.numTargets = 0;
        for (int i = 0; i < NUM_DRONES; i++) {
            this.drones[i] = new Drone();
        }
    }
    
    public World(JFrame field)
    {
        StringTokenizer input;
        try {
            this.in = new BufferedReader(new FileReader(files[fileIndex]));
            input = new StringTokenizer(in.readLine());
        } catch (IOException e) {
            System.out.println("File error - " + e.getMessage());
            System.exit(0);
            this.in = null;
            input = new StringTokenizer("");
        }
        this.xLength = Double.parseDouble(input.nextToken());
        this.yLength = Double.parseDouble(input.nextToken());
        this.zLength = Double.parseDouble(input.nextToken());
        this.windSpeed = Double.parseDouble(input.nextToken());
        this.windHeading = HEADING.values()[Integer.parseInt(input.nextToken())];
        this.T = Double.parseDouble(input.nextToken());
        this.NUM_DRONES = Integer.parseInt(input.nextToken());
        this.NUM_SIMULATION_CYCLES = Integer.parseInt(input.nextToken());
        this.drones = new Drone[NUM_DRONES];
        this.turn = 0;
        this.field = field;
        this.numTargets = 0;
        for (int i = 0; i < NUM_DRONES; i++) {
            this.drones[i] = new Drone();
        }
    }    
    
    /**
     * Where all the information gets updated
     **/
    public boolean update() throws IOException
    {
        if (turn >= NUM_SIMULATION_CYCLES) {
            return false;
        }
        StringTokenizer input = new StringTokenizer(in.readLine());
        numTargets = Integer.parseInt(input.nextToken());
        targetXPos = new double[numTargets];
        targetYPos = new double[numTargets];
        targetZPos = new double[numTargets];
        targetRadii = new double[numTargets];
        for (int i = 0; i < numTargets; i++) {
            input = new StringTokenizer(in.readLine());
            targetXPos[i] = Double.parseDouble(input.nextToken());
            targetYPos[i] = Double.parseDouble(input.nextToken());
            targetZPos[i] = Double.parseDouble(input.nextToken());
            targetRadii[i] = Double.parseDouble(input.nextToken());
        }
        for (int i = 0; i < NUM_DRONES; i++) {
            input = new StringTokenizer(in.readLine());
            //input.nextToken();
            //input.nextToken(); //skip two tokens
            drones[i].update(Double.parseDouble(input.nextToken()), //xPos
                             Double.parseDouble(input.nextToken()), //yPos
                             Double.parseDouble(input.nextToken()), //zPos
                             //Double.parseDouble(input.nextToken()), //xVel
                             //Double.parseDouble(input.nextToken()), //yVel
                             //Double.parseDouble(input.nextToken()), //zVel
                             //Double.parseDouble(input.nextToken()), //xAccel
                             //Double.parseDouble(input.nextToken()), //yAccel
                             //Double.parseDouble(input.nextToken()), //zAccel
                             //Double.parseDouble(input.nextToken()), //batteryLife
                             Integer.parseInt(input.nextToken()));  //alive
                             
        }
        
        turn++;
        return true;
    }
    
    public int getTurn() { return turn; }
    
    public Drone[] getDrones() { return drones; }
    
    public int getNumDeadDrones() {
        int sum = 0;
        for (int i = 0; i < drones.length; i++) {
            sum += 1 - drones[i].alive;
        }
        return sum;
    }
    
    public void close() throws IOException {
        in.close();
    }
}