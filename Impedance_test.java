package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

/**
 * Simple implementation of a compliant up and down movement.
 * The robot will move in a vertical pattern while adapting to external forces.
 */
public class Impedance_test extends RoboticsAPIApplication {
    // Robot declaration
    private LBR lbr;
    
    // Motion parameters
    private static final double VELOCITY_FACTOR = 0.25; // Relative velocity (25%)
    private static final double VERTICAL_DISTANCE = 200.0; // mm of up/down movement
    
    // Impedance control parameters - lower values = more compliant
    private static final double STIFFNESS_X = 500; // N/m
    private static final double STIFFNESS_Y = 500; // N/m
    private static final double STIFFNESS_Z = 500;  // N/m - Lower for vertical axis
    private static final double STIFFNESS_ROT = 300; // Nm/rad for rotational stiffness
    
    @Override
    public void initialize() {
        // Initialize robot
        lbr = getContext().getDeviceFromType(LBR.class);
    }

    @Override
    public void run() {
        // Define start position in joint space
        double[] startPosition = new double[]{
            Math.toRadians(-10), Math.toRadians(-90), Math.toRadians(0), 
            Math.toRadians(70), Math.toRadians(0), Math.toRadians(70), 
            Math.toRadians(90)
        };
        
        // Move to start position using PTP motion
        getLogger().info("Moving to start position");
        PTP ptpToStartPosition = ptp(startPosition);
        ptpToStartPosition.setJointVelocityRel(VELOCITY_FACTOR);
        lbr.move(ptpToStartPosition);
        
        // Create impedance control mode for compliant motion
        CartesianImpedanceControlMode impedanceControlMode = createImpedanceControlMode();
        
        // Get current position as reference
        Frame currentFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
        
        // Perform 5 cycles of up/down movements
        for (int i = 0; i < 5; i++) {
            // Calculate top and bottom positions
            Frame topPosition = currentFrame.copy();
            Frame bottomPosition = currentFrame.copy();
            
            topPosition.setZ(currentFrame.getZ() + VERTICAL_DISTANCE*3);
            bottomPosition.setZ(currentFrame.getZ() - VERTICAL_DISTANCE/2);
            
            // Move up with compliance
            getLogger().info("Moving up - cycle " + (i+1));
            CartesianPTP ptpToTopPosition = ptp(topPosition);
            ptpToTopPosition.setJointVelocityRel(VELOCITY_FACTOR);
            ptpToTopPosition.setMode(impedanceControlMode);
            lbr.move(ptpToTopPosition);
            
            // Read external torque after moving up
            readAndLogExternalTorque();
            
            // Move down with compliance
            getLogger().info("Moving down - cycle " + (i+1));
            CartesianPTP ptpToBottomPosition = ptp(bottomPosition);
            ptpToBottomPosition.setJointVelocityRel(VELOCITY_FACTOR);
            ptpToBottomPosition.setMode(impedanceControlMode);
            lbr.move(ptpToBottomPosition);
            
            // Read external torque after moving down
            readAndLogExternalTorque();
        }
        
        getLogger().info("Completed simple compliant motion");
    }
    
    /**
     * Reads and logs the external torques from the robot's sensors
     */
    private void readAndLogExternalTorque() {
        TorqueSensorData externalData = lbr.getExternalTorque();
        double[] torqueValues = externalData.getTorqueValues();
        
        // Build string representation of torque values
        StringBuilder torqueMessage = new StringBuilder("External torques: ");
        for (int i = 0; i < torqueValues.length; i++) {
            torqueMessage.append("J").append(i+1).append("=").append(String.format("%.4f", torqueValues[i]));
            if (i < torqueValues.length - 1) {
                torqueMessage.append("; ");
            }
        }
        
        getLogger().info(torqueMessage.toString());
    }
    
    /**
     * Creates a basic impedance control mode with appropriate stiffness settings.
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