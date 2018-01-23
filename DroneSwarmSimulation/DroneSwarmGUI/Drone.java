public class Drone {
    public double xPos, yPos, zPos;
    public double xVel, yVel, zVel;
    public double xAccel, yAccel, zAccel;
    public double batteryLife;
    public int alive;
    
    /*
    public Drone(double xPos, double yPos, double zPos, double xVel, double yVel, double zVel, double batteryLife, int alive) {
        update(xPos, yPos, zPos, xVel, yVel, zVel, batteryLife, alive);
    }
    */
   
   public void update(double xPos, double yPos, double zPos, int alive) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.zPos = zPos;
        this.alive = alive;
    }
    
    public void update(double xPos, double yPos, double zPos, double xVel, double yVel, double zVel, 
                       double xAccel, double yAccel, double zAccel, double batteryLife, int alive) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.zPos = zPos;
        this.xVel = xVel;
        this.yVel = yVel;
        this.zVel = zVel;
        this.xAccel = xAccel;
        this.yAccel = yAccel;
        this.zAccel = zAccel;
        this.batteryLife = batteryLife;
        this.alive = alive;
    }
}