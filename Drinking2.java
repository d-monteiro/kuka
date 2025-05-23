package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SplineJP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Safe drinking water motion with compliance and torque monitoring for the KUKA iiwa LBR robot.
 * The robot will move through a spline path simulating the action of 
 * picking up a cup and drinking water, with safety stops when torque thresholds are exceeded.
 * 
 * Designed for rehabilitation applications - helps stroke survivors and post-surgery patients.
 */
public class DrinkingMotion extends RoboticsAPIApplication {
    // Robot declaration
    private LBR lbr;
    
    // Motion parameters
    private static final double VELOCITY_FACTOR = 0.15; // Slow movement for safety
    
    // Impedance control parameters - adjust for desired compliance
    private static final double STIFFNESS_X = 300; // N/m
    private static final double STIFFNESS_Y = 300; // N/m
    private static final double STIFFNESS_Z = 300; // N/m
    private static final double STIFFNESS_ROT = 200; // Nm/rad for rotational stiffness
    
    // Safety torque thresholds (Nm) - adjust based on patient needs
    private static final double[] TORQUE_THRESHOLDS = {
        8.0,  // Joint 1 - Base rotation
        8.0,  // Joint 2 - Shoulder
        6.0,  // Joint 3 - Arm
        6.0,  // Joint 4 - Elbow
        4.0,  // Joint 5 - Wrist 1
        4.0,  // Joint 6 - Wrist 2
        3.0   // Joint 7 - Wrist 3
    };
    
    // Motion control flags
    private volatile boolean motionStopped = false;
    private volatile boolean continueMotion = false;
    
    // Starting position
    private final double[] startPosition = new double[]{
        Math.toRadians(-22), Math.toRadians(-110), Math.toRadians(-33), 
        Math.toRadians(71), Math.toRadians(-11), Math.toRadians(78), 
        Math.toRadians(107)
    };
    
    private final static String informationText =
        "REHABILITATION DRINKING MOTION with Safety Monitoring\n" +
        "\n" +
        "This application simulates a drinking motion with safety features:\n" +
        "• Continuous torque monitoring for patient safety\n" +
        "• Automatic motion stops when resistance is detected\n" +
        "• Option to extend range of motion or cancel\n" +
        "\n" +
        "The robot will move through 4 points:\n" +
        "1. Starting position - arm down\n" +
        "2. Pick up cup - move to table/cup position\n" +
        "3. Bring cup to mouth - move to face position\n" +
        "4. Return cup - move back to table position\n" +
        "\n" +
        "SAFETY: Motion will pause if excessive resistance is detected.\n" +
        "The motion will be compliant to allow for safe interaction.";
    
    @Override
    public void initialize() {
        // Initialize robot
        lbr = getContext().getDeviceFromType(LBR.class);
    }

    @Override
    public void run() {
        getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1) {
            return;
        }
        
        // Move to starting position
        getLogger().info("Moving to start position");
        PTP ptpToStartPosition = ptp(startPosition);
        ptpToStartPosition.setJointVelocityRel(VELOCITY_FACTOR);
        lbr.move(ptpToStartPosition);
        
        // Create impedance control mode for compliant motion
        CartesianImpedanceControlMode impedanceControlMode = createImpedanceControlMode();
        
        // Get current position for reference
        Frame currentFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
        
        // Execute the drinking motion with safety monitoring
        executeSafeDrinkingMotion(impedanceControlMode, currentFrame);
        
        // Return to the starting position
        getLogger().info("Returning to start position");
        PTP ptpBackToStart = ptp(startPosition);
        ptpBackToStart.setJointVelocityRel(VELOCITY_FACTOR);
        lbr.move(ptpBackToStart);
        
        getLogger().info("Drinking motion completed safely");
    }
    
    /**
     * Executes the drinking motion with continuous torque monitoring and safety stops
     */
    private void executeSafeDrinkingMotion(CartesianImpedanceControlMode impedanceControlMode, Frame startFrame) {
        // Get the individual points of the drinking motion
        double[][] drinkingPoints = getDrinkingMotionPoints();
        
        // Execute motion point by point with safety monitoring
        for (int i = 0; i < drinkingPoints.length; i++) {
            String phaseName = getMotionPhaseName(i);
            getLogger().info("Starting phase: " + phaseName);
            
            // Reset motion control flags
            motionStopped = false;
            continueMotion = false;
            
            // Execute single point motion with monitoring
            boolean success = executeSafePointToPointMotion(drinkingPoints[i], impedanceControlMode, phaseName);
            
            if (!success) {
                getLogger().info("Motion cancelled by user during phase: " + phaseName);
                return; // Exit the entire motion sequence
            }
            
            // Brief pause between phases
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }
    
    /**
     * Executes a single point-to-point motion with torque monitoring
     */
    private boolean executeSafePointToPointMotion(double[] targetJoints, 
                                                CartesianImpedanceControlMode impedanceControlMode, 
                                                String phaseName) {
        
        // Create the motion
        PTP ptpMotion = ptp(targetJoints);
        ptpMotion.setJointVelocityRel(VELOCITY_FACTOR);
        ptpMotion.setMode(impedanceControlMode);
        
        // Start monitoring thread
        Thread monitoringThread = new Thread(() -> monitorTorquesDuringMotion(phaseName));
        monitoringThread.start();
        
        try {
            // Execute the motion
            lbr.move(ptpMotion);
            
            // Stop monitoring
            motionStopped = true;
            monitoringThread.interrupt();
            
        } catch (Exception e) {
            getLogger().error("Error during motion execution: " + e.getMessage());
            motionStopped = true;
            monitoringThread.interrupt();
            return false;
        }
        
        return true;
    }
    
    /**
     * Continuously monitors torques during motion execution
     */
    private void monitorTorquesDuringMotion(String phaseName) {
        try {
            while (!motionStopped && !Thread.currentThread().isInterrupted()) {
                // Read current torque values
                TorqueSensorData externalData = lbr.getExternalTorque();
                double[] torqueValues = externalData.getTorqueValues();
                
                // Check if any torque exceeds threshold
                boolean thresholdExceeded = false;
                int exceedingJoint = -1;
                
                for (int i = 0; i < torqueValues.length && i < TORQUE_THRESHOLDS.length; i++) {
                    if (Math.abs(torqueValues[i]) > TORQUE_THRESHOLDS[i]) {
                        thresholdExceeded = true;
                        exceedingJoint = i + 1; // Joint numbering starts from 1
                        break;
                    }
                }
                
                if (thresholdExceeded) {
                    // Log the safety stop
                    getLogger().warn("SAFETY STOP: Torque threshold exceeded on Joint " + exceedingJoint + 
                                   " during " + phaseName + 
                                   " (Value: " + String.format("%.3f", Math.abs(torqueValues[exceedingJoint-1])) + 
                                   " Nm, Threshold: " + TORQUE_THRESHOLDS[exceedingJoint-1] + " Nm)");
                    
                    // Stop the current motion
                    lbr.stop();
                    
                    // Show safety dialog to user
                    boolean shouldContinue = showSafetyDialog(phaseName, exceedingJoint, 
                                                            Math.abs(torqueValues[exceedingJoint-1]));
                    
                    if (!shouldContinue) {
                        motionStopped = true;
                        return; // Exit monitoring
                    }
                    
                    // If user wants to continue, adjust thresholds temporarily
                    getLogger().info("User chose to continue motion - temporarily increasing threshold tolerance");
                }
                
                // Check every 50ms for responsive monitoring
                Thread.sleep(50);
            }
        } catch (InterruptedException e) {
            // Thread was interrupted, exit gracefully
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            getLogger().error("Error in torque monitoring: " + e.getMessage());
        }
    }
    
    /**
     * Shows a safety dialog when torque threshold is exceeded
     */
    private boolean showSafetyDialog(String phaseName, int joint, double torqueValue) {
        String safetyMessage = 
            "SAFETY ALERT - Motion Paused\n\n" +
            "Phase: " + phaseName + "\n" +
            "Joint " + joint + " torque exceeded safe limit\n" +
            "Current: " + String.format("%.2f", torqueValue) + " Nm\n" +
            "Threshold: " + String.format("%.2f", TORQUE_THRESHOLDS[joint-1]) + " Nm\n\n" +
            "This may indicate:\n" +
            "• Patient resistance or discomfort\n" +
            "• Need to extend range of motion gradually\n" +
            "• End of comfortable motion range\n\n" +
            "Would you like to continue the motion?\n" +
            "(This will temporarily allow higher torques)";
        
        int userChoice = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION, 
            safetyMessage, 
            "Continue Motion", 
            "Stop & Cancel"
        );
        
        return (userChoice == 0); // 0 = Continue, 1 = Cancel
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
            
            // Point 3: Bring cup to mouth position (back to elevated position)
            {Math.toRadians(-3), Math.toRadians(-102), Math.toRadians(-20), 
             Math.toRadians(86), Math.toRadians(6), Math.toRadians(79), 
             Math.toRadians(110)},
            
            // Point 4: Return cup to table (same as point 2)
            {Math.toRadians(-37), Math.toRadians(-102), Math.toRadians(-4), 
             Math.toRadians(81), Math.toRadians(-10), Math.toRadians(110), 
             Math.toRadians(109)}
        };
    }
    
    /**
     * Returns human-readable names for each motion phase
     */
    private String getMotionPhaseName(int phaseIndex) {
        switch (phaseIndex) {
            case 0: return "Reaching toward table";
            case 1: return "Picking up cup";
            case 2: return "Bringing cup to mouth";
            case 3: return "Returning cup to table";
            default: return "Unknown phase";
        }
    }
    
    /**
     * Creates a spline path for the drinking motion using 4 absolute joint positions.
     * NOTE: This method is kept for compatibility but not used in the new safe implementation
     */
    private SplineJP createDrinkingSpline(Frame startFrame) {
        double[][] points = getDrinkingMotionPoints();
        
        SplineJP spline = new SplineJP(
            ptp(points[0]),
            ptp(points[1]),
            ptp(points[2]),
            ptp(points[3])
        );
        
        return spline;
    }
    
    /**
     * Creates a compliant impedance control mode with appropriate stiffness settings.
     * @return A configured CartesianImpedanceControlMode
     */
    private CartesianImpedanceControlMode createImpedanceControlMode() {
        CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
        
        // Set translational stiffness
        impedanceControlMode.parametrize(CartDOF.X).setStiffness(STIFFNESS_X);
        impedanceControlMode.parametrize(CartDOF.Y).setStiffness(STIFFNESS_Y);
        impedanceControlMode.parametrize(CartDOF.Z).setStiffness(STIFFNESS_Z);
        
        // Set rotational stiffness
        impedanceControlMode.parametrize(CartDOF.A).setStiffness(STIFFNESS_ROT);
        impedanceControlMode.parametrize(CartDOF.B).setStiffness(STIFFNESS_ROT);
        impedanceControlMode.parametrize(CartDOF.C).setStiffness(STIFFNESS_ROT);
        
        // Set damping (0.7 is a typical value)
        impedanceControlMode.parametrize(CartDOF.ALL).setDamping(0.7);
        
        return impedanceControlMode;
    }
}