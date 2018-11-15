package org.firstinspires.ftc.teamcode.Control;

public class ENUMS {

    public enum DriveTypes{
        MECANUM, STANDARD, TANK, TANKWITHMECANUM, SIXWHEEL
    }

    public enum AutoStates{
        START, NAVTOSAMPLE1, MOVETOTURN2, NAVTOSAMPLE2, FINDGOLD, TURNTOGOLD, STOREGOLDPOS, HITGOLD, FINDWALL, DROPTM, FINDCRATER, PARKANDEND, TESTGYRO, END
    }

    public enum GoldPosition{
        LEFT, CENTER, RIGHT, UNKNOWN
    }

    public enum AutoTypes{
        BLUECRATER, BLUETM, REDCRATER, REDTM
    }


}
