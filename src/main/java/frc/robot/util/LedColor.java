// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org


package frc.robot.util;

import edu.wpi.first.wpilibj.util.Color;

public enum LedColor {
    kBlack(0, 0, 0),
    kRed(255, 0, 0),
    kGreen(0, 255, 0),
    kBlue(0, 0, 255),
    kYellow(255, 255, 0),
    kPurple(255, 0, 255),
    kCyan(0, 255, 255),
    kOrange(255, 119, 0);

    private int r;
    private int g;
    private int b;

    private LedColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public int getRed() {
        return r;
    }

    public int getGreen() {
        return g;
    }

    public int getBlue() {
        return b;
    }

    public int[] getAll(){
        return(new int[]{r,g,b});
    }

    public Color getColor(){
        return(new Color(r, g, b));
    }
}