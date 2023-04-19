// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;

public enum MotorFault{
    
    kBrownout(0x01),
    kCanMismatch(0x02),
    kEepromCrc(0x04),
    kEepromRead(0x08),
    kHardLimitForward(0x10),
    kHardLimitReverse(0x20),
    kSoftLimitForward(0x40),
    kSoftLimitReverse(0x80),
    kStall(0x100),
    kSupplyVoltage(0x200);

    private final int value;

    MotorFault(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }

}

