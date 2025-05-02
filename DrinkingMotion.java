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
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Simple drinking water motion with compliance for the KUKA iiwa LBR robot.
 * The robot will move through a spline path simulating the action of 
 * picking up a cup and drinking water.
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
    
    // Starting position
    private final double[] startPosition = new double[]{
        Math.toRadians(-22), Math.toRadians(-110), Math.toRadians(-33), 
        Math.toRadians(71), Math.toRadians(-11), Math.toRadians(78), 
        Math.toRadians(107)
    };
    
    private final static String informationText =
        "This application simulates a drinking motion with an end effector attached to your elbow.\n" +
        "\n" +
        "The robot will move through 4 points:\n" +
        "1. Starting position - arm down\n" +
        "2. Pick up cup - move to table/cup position\n" +
        "3. Bring cup to mouth - move to face position\n" +
        "4. Return cup - move back to table position\n" +
        "\n" +
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
        
        // Create drinking motion spline
        SplineJP drinkingSpline = createDrinkingSpline(currentFrame);
        drinkingSpline.setJointVelocityRel(VELOCITY_FACTOR);
        
        // Apply impedance control to the spline
        drinkingSpline.setMode(impedanceControlMode);
        
        // Execute the drinking motion
        getLogger().info("Executing drinking motion");
        lbr.move(drinkingSpline);
        
        // Return to the starting position
        getLogger().info("Returning to start position");
        PTP ptpBackToStart = ptp(startPosition);
        ptpBackToStart.setJointVelocityRel(VELOCITY_FACTOR);
        lbr.move(ptpBackToStart);
        
        getLogger().info("Drinking motion completed");
    }
    
    /**
     * Creates a spline path for the drinking motion using 4 absolute joint positions.
     * @param startFrame - The current position of the robot (not used in this version)
     * @return A spline representing the drinking motion
     */
    private SplineJP createDrinkingSpline(Frame startFrame) {
        // Point 1: Start position (defined using joint angles)
        double[] point1 = new double[]{
            Math.toRadians(-3), Math.toRadians(-102), Math.toRadians(-20), 
            Math.toRadians(86), Math.toRadians(6), Math.toRadians(79), 
            Math.toRadians(110)
        };
        
        // Point 2: Pick up cup from table (reach down)
        double[] point2 = new double[]{
            Math.toRadians(-37), Math.toRadians(-102), Math.toRadians(-4), 
            Math.toRadians(81), Math.toRadians(-10), Math.toRadians(110), 
            Math.toRadians(109)
        };
        
        // Point 3: Bring cup to mouth position
        double[] point3 = point1.clone();
        
        // Point 4: Return cup to table (same as point 2)
        double[] point4 = point2.clone();
        
        // Create a SplineJP for joint position spline motion
        SplineJP spline = new SplineJP(
            ptp(point1),
            ptp(point2),
            ptp(point3),
            ptp(point4)
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