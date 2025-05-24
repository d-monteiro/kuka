package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.SplineJP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;

/**
 * Simple drinking water motion with compliance for the KUKA iiwa LBR robot.
 * Modified to send torque data via UDP and be controlled externally.
 */
public class DrinkingData extends RoboticsAPIApplication {
    // Robot declaration
    private LBR lbr;
    
    // Network parameters
    private static final String PC_IP = "172.31.1.148"; // PC's IP address
    private static final int UDP_PC_PORT = 30001;       // PC's UDP port for torque data
    private static final int UDP_LOCAL_PORT = 30001;   // Local UDP port on robot
    private static final int UDP_MOTION = 30002;
    
    // Motion parameters - now modifiable via external input
    private static final double VELOCITY_FACTOR = 0.15; // Slow movement for safety
    
    // Default impedance control parameters - will be updated from GUI
    private double stiffnessX = 300; // N/m
    private double stiffnessY = 300; // N/m
    private double stiffnessZ = 300; // N/m
    private double stiffnessRot = 200; // Nm/rad for rotational stiffness
    
    // Motion control flags
    private volatile boolean stopMotion = false;
    private volatile boolean motionActive = false;
    private volatile boolean executeMotion = false;
    
    // Starting position	
    private final double[] startPosition = new double[]{
        Math.toRadians(-22), Math.toRadians(-110), Math.toRadians(-33), 
        Math.toRadians(71), Math.toRadians(-11), Math.toRadians(78), 
        Math.toRadians(107)
    };
    
    // UDP socket for communication
    private DatagramSocket udpSocket;
    
    @Override
    public void initialize() {
        // Initialize robot
        lbr = getContext().getDeviceFromType(LBR.class);
        
        try {
            // Create UDP socket for torque data transmission
            udpSocket = new DatagramSocket(UDP_LOCAL_PORT);
            getLogger().info("UDP socket created for torque data transmission");
        } catch (Exception e) {
            getLogger().error("Failed to create UDP socket: " + e.getMessage());
        }
    }

    @Override
    public void run() {
        getLogger().info("DrinkingMotion application started - waiting for external control");
        
        // Move to starting position
        getLogger().info("Moving to start position");
        PTP ptpToStartPosition = ptp(startPosition);
        ptpToStartPosition.setJointVelocityRel(VELOCITY_FACTOR);
        lbr.move(ptpToStartPosition);
        
        // Start monitoring thread using explicit Runnable
        Thread torqueThread = new Thread(new Runnable() {
            @Override
            public void run() {
                transmitTorqueData();
            }
        });
        torqueThread.start();
        
        Thread commandThread = new Thread(new Runnable() {
            @Override
            public void run() {
                receiveCommands();
            }
        });
        commandThread.setDaemon(true);
        commandThread.start();
        
        // Main control loop - wait for external commands or motion execution
        try {
            while (!Thread.currentThread().isInterrupted()) {
                // This application now runs continuously, controlled by external GUI
                // The actual drinking motion will be triggered externally
                
                // Check if motion should be executed (this would be set by external command)
                if (shouldExecuteMotion()) {
                    executeRehabilitationMotion();
                }
                
                Thread.sleep(100); // Check every 100ms
            }
        } catch (InterruptedException e) {
            getLogger().info("Application interrupted");
        } finally {
            // Cleanup
            if (udpSocket != null && !udpSocket.isClosed()) {
                udpSocket.close();
            }
        }
    }
    
    /**
     * Executes the rehabilitation drinking motion
     */
    private void executeRehabilitationMotion() {
        if (motionActive) {
            return; // Motion already running
        }
        
        motionActive = true;
        stopMotion = false;
        
        try {
            getLogger().info("Starting rehabilitation drinking motion");
            
            // Create impedance control mode with current stiffness settings
            CartesianImpedanceControlMode impedanceControlMode = createImpedanceControlMode();
            
            // Get current position for reference
            Frame currentFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
            
            // Execute motion point by point for better control
            double[][] motionPoints = getDrinkingMotionPoints();
            String[] phaseNames = {"Reaching toward table", "Picking up cup", 
                                  "Bringing cup to mouth", "Returning cup to table"};
            
            for (int i = 0; i < motionPoints.length && !stopMotion; i++) {
                getLogger().info("Phase " + (i+1) + ": " + phaseNames[i]);
                
                PTP ptpMotion = ptp(motionPoints[i]);
                ptpMotion.setJointVelocityRel(VELOCITY_FACTOR);
                ptpMotion.setMode(impedanceControlMode);
                
                // Execute motion with stop check
                lbr.move(ptpMotion);
                
                if (stopMotion) {
                    getLogger().info("Motion stopped by external command");
                    break;
                }
                
                // Brief pause between phases
                Thread.sleep(500);
            }
            
            if (!stopMotion) {
                // Return to starting position
                getLogger().info("Returning to start position");
                PTP ptpBackToStart = ptp(startPosition);
                ptpBackToStart.setJointVelocityRel(VELOCITY_FACTOR);
                lbr.move(ptpBackToStart);
                getLogger().info("Drinking motion completed");
            }
            
        } catch (Exception e) {
            getLogger().error("Error during motion execution: " + e.getMessage());
        } finally {
            motionActive = false;
        }
    }
    
    /**
     * Continuously transmits torque data to PC
     */
    private void transmitTorqueData() {
        InetSocketAddress pcAddress = new InetSocketAddress(PC_IP, UDP_PC_PORT);
        
        try {
            while (!Thread.currentThread().isInterrupted()) {
                // Get torque values
                TorqueSensorData externalData = lbr.getExternalTorque();
                double[] torqueValues = externalData.getTorqueValues();
                
                // Convert torque values to string with motion status
                StringBuilder torqueMessage = new StringBuilder();
                
                // Add motion status
                torqueMessage.append(motionActive ? "MOVING" : "IDLE").append(";");
                
                // Add torque values
                for (double torque : torqueValues) {
                    torqueMessage.append(String.format("%.4f;", torque));
                }
                
                // Remove trailing semicolon
                if (torqueMessage.length() > 0) {
                    torqueMessage.setLength(torqueMessage.length() - 1);
                }
                
                // Send data
                byte[] sendData = torqueMessage.toString().getBytes("UTF-8");
                udpSocket.send(new DatagramPacket(sendData, sendData.length, pcAddress));
                
                // Send at 10 Hz rate
                Thread.sleep(100);
            }
        } catch (Exception e) {
            getLogger().error("Error in torque transmission: " + e.getMessage());
        }
    }
    
    /**
     * Check if motion should be executed
     */
    private boolean shouldExecuteMotion() {
        if (executeMotion && !motionActive) {
            executeMotion = false; // Reset flag
            return true;
        }
        return false;
    }
    
    /**
     * Update stiffness parameters from external GUI
     */
    public void updateStiffness(double x, double y, double z, double rot) {
        this.stiffnessX = x;
        this.stiffnessY = y;
        this.stiffnessZ = z;
        this.stiffnessRot = rot;
        getLogger().info("Stiffness updated: X=" + x + ", Y=" + y + ", Z=" + z + ", Rot=" + rot);
    }
    
    /**
     * Returns the joint positions for each phase of the drinking motion
     */
    private double[][] getDrinkingMotionPoints() {
        return new double[][] {
            // Point 1: Start position (reach towards table)
            {Math.toRadians(-3), Math.toRadians(-102), Math.toRadians(-20), 
             Math.toRadians(86), Math.toRadians(6), Math.toRadians(79), 
             Math.toRadians(110)},
            
            // Point 2: Pick up cup from table (reach down)
            {Math.toRadians(-37), Math.toRadians(-102), Math.toRadians(-4), 
             Math.toRadians(81), Math.toRadians(-10), Math.toRadians(110), 
             Math.toRadians(109)},
            
            // Point 3: Bring cup to mouth position
            {Math.toRadians(-3), Math.toRadians(-102), Math.toRadians(-20), 
             Math.toRadians(86), Math.toRadians(6), Math.toRadians(79), 
             Math.toRadians(110)},
            
            // Point 4: Return cup to table
            {Math.toRadians(-37), Math.toRadians(-102), Math.toRadians(-4), 
             Math.toRadians(81), Math.toRadians(-10), Math.toRadians(110), 
             Math.toRadians(109)}
        };
    }
    
    /**
     * Creates a compliant impedance control mode with current stiffness settings.
     */
    private CartesianImpedanceControlMode createImpedanceControlMode() {
        CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
        
        // Set translational stiffness (using current values)
        impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
        impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
        impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
        
        // Set rotational stiffness
        impedanceControlMode.parametrize(CartDOF.A).setStiffness(stiffnessRot);
        impedanceControlMode.parametrize(CartDOF.B).setStiffness(stiffnessRot);
        impedanceControlMode.parametrize(CartDOF.C).setStiffness(stiffnessRot);
        
        // Set damping
        impedanceControlMode.parametrize(CartDOF.ALL).setDamping(0.7);
        
        return impedanceControlMode;
    }
    
    /**
     * Receives commands from the PC GUI using the existing UDP socket setup
     */
    private void receiveCommands() {
        try {
            // Create a separate socket for receiving commands on the same port as main communication
            DatagramSocket commandSocket = new DatagramSocket(UDP_MOTION);
            commandSocket.setSoTimeout(100); // 100ms timeout
            
            byte[] buffer = new byte[1024];
            
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    commandSocket.receive(packet);
                    
                    String message = new String(packet.getData(), 0, packet.getLength(), "UTF-8");
                    String[] parts = message.split(";");
                    
                    if (parts.length >= 4) {
                        String command = parts[2]; // Command is in 3rd position
                        String value = parts[3];   // Value is in 4th position
                        
                        if ("START_MOTION".equals(command) && "true".equals(value)) {
                            getLogger().info("Motion start command received");
                            executeMotion = true;
                        }
                        else if ("Emergency_Stop".equals(command) && "true".equals(value)) {
                            getLogger().info("Emergency stop command received");
                        }
                    }
                    
                } catch (java.net.SocketTimeoutException e) {
                    // Timeout is normal, continue loop
                    continue;
                } catch (Exception e) {
                    // Log error but continue
                    continue;
                }
            }
            
            commandSocket.close();
        } catch (Exception e) {
            getLogger().error("Command reception error: " + e.getMessage());
        }
    }
}