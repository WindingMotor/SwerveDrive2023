// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;

public enum Section {

    kAll(0,109),
    kLeft(0,45),
    kRight(46,109),
    kBottomLeft(0, 10),
    kBottomRight(99,109),
    kTopLeft(35,45),
    kTopRight(46, 56);

    private int start;
    private int end;

    private Section(int start, int end) {
        this.start = start;
        this.end = end;

    }

    public int getStart() {
        return start;
    }

    public int getEnd() {
        return end;
    }

}