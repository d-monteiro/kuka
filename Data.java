package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.motionModel.PTP;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;

public class Data extends RoboticsAPIApplication {
    private LBR robot;
    private static final String PC_IP = "172.31.1.148"; // PC's IP address
    private static final int UDP_PC_PORT = 30001;       // PC's UDP port
    private static final int UDP_LOCAL_PORT = 30001;   // Local UDP port on robot

    @Override
    public void initialize() {
        robot = getContext().getDeviceFromType(LBR.class);
    }

    @Override
    public void run() {
    	
    	PTP ptpToTransportPosition = ptp(0, Math.toRadians(25), 0, Math.toRadians(90), 0, 0, 0);
		ptpToTransportPosition.setJointVelocityRel(0.25);
		robot.move(ptpToTransportPosition);
		getLogger().info("In run(), after motion");
		
        DatagramSocket udpSocket = null;
        try {
            // Create a UDP socket
            udpSocket = new DatagramSocket(UDP_LOCAL_PORT);
            getLogger().info("UDP socket created.");

            // Define PC address
            InetSocketAddress pcAddress = new InetSocketAddress(PC_IP, UDP_PC_PORT);

            // Continuously send torque values
            while (true) {
                // Get torque values
                TorqueSensorData externalData = robot.getExternalTorque();
                double[] torqueValues = externalData.getTorqueValues();

                // Convert torque values to a string for transmission
                StringBuilder torqueMessage = new StringBuilder();
                for (double torque : torqueValues) {
                    torqueMessage.append(String.format("%.4f;", torque)); // Send with 4 decimal precision
                }

                // Remove the trailing semicolon
                if (torqueMessage.length() > 0) {
                    torqueMessage.setLength(torqueMessage.length() - 1);
                }

                // Send torque values to PC via UDP
                byte[] sendData = torqueMessage.toString().getBytes("UTF-8");
                udpSocket.send(new DatagramPacket(sendData, sendData.length, pcAddress));
                getLogger().info("Torque values sent: " + torqueMessage);

                // Control the transmission rate (e.g., 10 Hz)
                Thread.sleep(100);
            }
        } catch (Exception e) {
            e.printStackTrace();
            getLogger().info("Error occurred in run method.");
        } finally {
            if (udpSocket != null && !udpSocket.isClosed()) {
                udpSocket.close();
                getLogger().info("UDP socket closed.");
            }
        }
    }
}
