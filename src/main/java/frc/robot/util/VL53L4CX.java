package frc.robot.util;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VL53L4CX extends SubsystemBase {

    /* 
    private static final int DEVICE_ADDRESS = 0x29;
    private static final int SHORT_RANGE_MODE_REGISTER_ADDRESS = 0x010B;
    private static final int SIGNAL_RATE_LIMIT_REGISTER_ADDRESS = 0x84;
    private static final int TIMING_BUDGET_REGISTER_ADDRESS = 0x81;

    private final I2C i2c;
    private final int timingBudget;

    public VL53L4CX(int timingBudget) {
        this.timingBudget = timingBudget;

        // Connect the sensor to the I2C port on the RoboRIO
        I2C.Port i2cPort = I2C.Port.kOnboard;
        i2c = new I2C(i2cPort, DEVICE_ADDRESS);

        // Configure the sensor
        configureSensor();

    }

    @Override
    public void periodic() {
        int distance = readWord(SHORT_RANGE_MODE_REGISTER_ADDRESS);
        System.out.println("Distance: " + distance + " mm");
    }

    public double getDistanceMM(){
        int distance = readWord(SHORT_RANGE_MODE_REGISTER_ADDRESS);
        return((double)distance);
    }

    public double getDistanceM(){
        return(getDistanceMM()/1000);
    }

    // Write a single byte
    private void writeByte(int registerAddress, int data) {
        i2c.write(registerAddress, data);
    }

    // Read a byte address
    private byte readByte(int registerAddress) {
        byte[] dataReceived = new byte[1];
        i2c.read(registerAddress, 1, dataReceived);
        return dataReceived[0];
    }

    // Write a word
    private void writeWord(int registerAddress, int data) {
        byte[] dataToSend = new byte[2];
        dataToSend[0] = (byte) ((data >> 8) & 0xFF);
        dataToSend[1] = (byte) (data & 0xFF);
        i2c.writeBulk(dataToSend);
    }

    // Read a word
    private int readWord(int registerAddress) {
        byte[] data = new byte[2];
        i2c.read(registerAddress, data.length, data);
        int value = ((data[1] & 0xFF) << 8) | (data[0] & 0xFF);
        return value;
    }

    // Send data to configure the sensor
    private void configureSensor() {

        // Configure sensor for short range mode
        writeByte(SHORT_RANGE_MODE_REGISTER_ADDRESS, 0x01);
        // Increase sensor range -> writeByte(SHORT_RANGE_MODE_REGISTER_ADDRESS, 0x02);
        writeByte(0x0120, (byte) 0x01);

        // Set timing budget and signal rate limit
        writeWord(TIMING_BUDGET_REGISTER_ADDRESS, timingBudget);
        writeByte(SIGNAL_RATE_LIMIT_REGISTER_ADDRESS, 0x00);
    }
*/
}
