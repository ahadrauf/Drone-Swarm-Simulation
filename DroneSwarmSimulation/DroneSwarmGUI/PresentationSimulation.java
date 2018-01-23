import java.awt.event.*;
import java.util.Date;
import javax.swing.*;
import java.awt.*;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

/*
import java.net.URL;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.Clip;
import javax.sound.sampled.*;
*/

public class PresentationSimulation extends JPanel
{
    private static boolean leftPressed = false;
    private static boolean rightPressed = false;
    private static JFrame worldScreen = null;
    private static boolean initializationComplete = false;
    private static World world = null;
    private static double playbackSpeed = 100;
    private static final int displayMode = 0; //0 = XY plane, 1 = XZ plane
    private static final boolean delay = false;
    private static final int delayAmountMillis = 1;
    
    public static void run()
    {
        KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyEventDispatcher()
        {
        @Override
            public boolean dispatchKeyEvent(KeyEvent ke)
            {
                synchronized (World.class)
                {
                    switch (ke.getID())
                    {
                    case KeyEvent.KEY_PRESSED:
                        if (ke.getKeyCode() == KeyEvent.VK_RIGHT)
                            rightPressed = true;
                        if (ke.getKeyCode() == KeyEvent.VK_LEFT)
                            leftPressed = true;
                        break;

                    case KeyEvent.KEY_RELEASED:
                        if (ke.getKeyCode() == KeyEvent.VK_RIGHT)
                            rightPressed = false;
                        if (ke.getKeyCode() == KeyEvent.VK_LEFT)
                            leftPressed = false;
                        break;
                    }
                }
                return false;
            }
        });
        startSimulation();
    }
    
    private static void startSimulation()
    {
        SwingUtilities.invokeLater(new Runnable() {
            public void run()
            {
                new PresentationSimulation().run();
            }
        }); 
    }
    
    private static void endSimulation() {
        worldScreen = null;
        Object[] options = {"OK"};
        String message = "Simulation Complete\nYou only destroyed " + world.getNumDeadDrones() + 
            "/" + world.getDrones().length + " drones!";
        JOptionPane.showOptionDialog(null, message, "Simulation Complete", 
                 JOptionPane.DEFAULT_OPTION, JOptionPane.WARNING_MESSAGE, 
                 null, options, options[0]);
        try {
            world.close();
        } catch (IOException e) {
            System.out.println("Something's gone wrong: " + e.getMessage());
        }
        System.exit(0);
    }
    
    public PresentationSimulation()
    {
        if (!initializationComplete) {
            initialize();
        }
        
        if (isLeftPressed())
            playbackSpeed /= 2.0;
        else if (isRightPressed())
            playbackSpeed *= 2.0;
        try {
            if (!world.update()) {
                endSimulation();
            };
        } catch (IOException e) {
            System.out.println("Something's gone wrong: " + e.getMessage());
        }
        
        worldScreen.repaint();
        /*
        if (world.getTurn() == 100) {
            try {
                TimeUnit.MILLISECONDS.sleep(1000);
            } catch (Exception e) {
                System.out.println("sleep error");
            }
        }
        */
        
        if (world.getTurn() == world.NUM_SIMULATION_CYCLES - 1) {            
            endSimulation();
        }
    }
    
    
    private void initialize() {
        initializationComplete = true;
        worldScreen = new JFrame("Simulate!");
        worldScreen.setSize(700, 750);
        worldScreen.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        worldScreen.setVisible(true);
        world = new World(worldScreen);
        worldScreen.add(new PresentationSimulation());
    }
    
    
    @Override
    public void paintComponent(Graphics g)
    {
        int i = 0;
        Graphics2D g2d = (Graphics2D) g;
        g2d.setColor(Color.BLACK);
        //Draw grid
        g2d.drawLine(350,0,350,670); //y/z-axis
        switch(displayMode) {
            case 0: g2d.drawString("y (m)", 320, 20); break;
            case 1: g2d.drawString("z (m)", 320, 20); break;
        }
        for (int j = -9; j <= 9; j++) {
            g2d.drawLine(340,(int)(670.0/20*(j+10)),360,(int)(670.0/20*(j+10))); //tick marks on the y-axis
            if (j == 0) { continue; } //don't write y/z = 0
            switch(displayMode) {
                case 0: g2d.drawString((int)(-world.yLength/20*j) + "", 365, (int)(670.0/20*(j+10)) + 2); break;
                case 1: g2d.drawString((int)(world.zLength/20*(10-j)) + "", 365, (int)(670.0/20*(j+10)) + 2); break;
            }
        }
        
        g2d.drawLine(0,670/2,700,670/2); //x-axis
        g2d.drawString("x (m)", 655, 670/2-15);
        for (int j = -9; j <= 9; j++) {
            g2d.drawLine(700/20*(j+10),670/2-10,700/20*(j+10),670/2+10); //tick marks on the x-axis
            if (j == 0) { continue; } //don't write x = 0
            g2d.drawString((int)(world.xLength/20*j) + "", 700/20*(j+10) - 20, 670/2+15);
        }
        
        int[] targetXPos = new int[world.numTargets];
        int[] targetYPos = new int[world.numTargets];
        int[] targetZPos = new int[world.numTargets];
        
        //Draw targets
        for (int j = 0; j < world.numTargets; j++) {
            int scaledXPos = (int)(world.targetXPos[j] / world.xLength * 700 + 350); //put smaller x at the left
            int scaledYPos = (int)((world.yLength/2 - world.targetYPos[j]) / world.yLength * 670); //put smaller y at the bottom
            int scaledZPos = (int)((world.zLength - world.targetZPos[j]) / world.zLength * 670); //put z = 0 on the bottom
            int scaledRadius = (int)(world.targetRadii[j] / world.yLength * 670);
            targetXPos[j] = scaledXPos;
            targetYPos[j] = scaledYPos;
            targetZPos[j] = scaledZPos;
            g2d.setColor(Color.BLACK);
            switch(displayMode) {
                case 0: g2d.fillOval(scaledXPos - scaledRadius, scaledYPos - scaledRadius,
                                     scaledRadius*2, scaledRadius*2); 
                        g2d.setColor(Color.GREEN);
                        g2d.drawString("T", (int)scaledXPos - 2, (int)scaledYPos + 5);
                        break;
                case 1: g2d.fillOval(scaledXPos - scaledRadius, scaledZPos - scaledRadius,
                                     scaledRadius*2, scaledRadius*2); 
                        g2d.setColor(Color.GREEN);
                        g2d.drawString("X", (int)scaledXPos - 2, (int)scaledZPos + 5);
                        break;
            }
            
        }
        
        for (Drone d : world.getDrones())
        {
            //Draw drones
            g2d.setColor(Color.RED);
            double scaledXPos = d.xPos / world.xLength * 700 + 350; //put smaller x at the left
            double scaledYPos = (world.yLength/2 - d.yPos) / world.yLength * 670; //put smaller y at the bottom
            double scaledZPos = (world.zLength - d.zPos) / world.zLength * 670; //put z = 0 on the bottom
            if (d.alive == 1) {
                switch(displayMode) {
                    case 0:
                        g2d.drawOval((int)scaledXPos, (int)scaledYPos, 25, 25);
                        g2d.setColor(Color.BLACK);
                        g2d.drawString("" + i, (int)scaledXPos + 7, (int)scaledYPos + 16);
                        break;
                    case 1:
                        g2d.drawOval((int)scaledXPos, (int)scaledZPos, 25, 25);
                        g2d.setColor(Color.BLACK);
                        g2d.drawString("" + i, (int)scaledXPos + 7, (int)scaledZPos + 16);
                        break;
                }
            } else {
                switch(displayMode) {
                    case 0:
                        g2d.drawString("X", (int)scaledXPos + 7, (int)scaledYPos + 16);
                        g2d.setColor(Color.BLACK);
                        g2d.drawString("" + i, (int)scaledXPos + 10, (int)scaledYPos + 16);
                        break;
                    case 1:
                        g2d.drawString("X", (int)scaledXPos + 7, (int)scaledZPos + 16);
                        g2d.setColor(Color.BLACK);
                        g2d.drawString("" + i, (int)scaledXPos + 10, (int)scaledZPos + 16);
                        break;
                }
            }   
            i++;
        }
        //draw time elapsed
        g2d.setColor(Color.BLACK);
        g2d.drawString((int)(world.T*world.getTurn()) + " s", 650, 20);
        if (delay) {
            try {
                TimeUnit.MILLISECONDS.sleep(delayAmountMillis);
            } catch (Exception e) {
                System.out.println("sleep error");
            }
        }
        
        //Draw shooter plane
        if (world.getTurn() > 3000 && world.getTurn() < 6500) {
            g2d.setColor(Color.BLACK);
            g2d.fillRect(600, 670/2-20, 100, 40);
            g2d.setColor(Color.WHITE);
            g2d.drawString("Missile", 620, 670/2-10);
            g2d.drawString("Launcher", 620, 670/2+5);
            g2d.setColor(Color.RED);
            for (int j = 0; j < world.numTargets; j++) {
                switch(displayMode) {
                    case 0: g2d.drawLine(600, 670/2, targetXPos[j], targetYPos[j]); break;
                    case 1: g2d.drawLine(600, 670/2, targetXPos[j], targetZPos[j]); break;
                }
            }
        }
        
        //Add subtitles
        if (world.getTurn() == 3) {
            //g2d.setColor(Color.RED);
            //g2d.fillRect(0, 0, 100, 100);
            //g2d.drawString("Drones are indicated by the red circles", 350, 670/2-120);
            
            try {
                TimeUnit.MILLISECONDS.sleep(10000);
            } catch (Exception e) {
                System.out.println("sleep error");
            }
        }
    }
    
    private static boolean isLeftPressed()
    {
        synchronized (Simulation.class)
        {
            return leftPressed;
        }
    }

    private static boolean isRightPressed()
    {
        synchronized (Simulation.class)
        {
            return rightPressed;
        }
    }
}