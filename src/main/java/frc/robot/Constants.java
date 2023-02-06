package frc.robot;

public class Constants {
    private Constants() {

    }

    public static final int LEFT_FRONT_PORT = 51;
    public static final int LEFT_BACK_PORT = 34;

    public static final int RIGHT_FRONT_PORT = 49;
    public static final int RIGHT_BACK_PORT = 46;

    public static final boolean LEFT_INVERTED = true;
    public static final boolean RIGHT_INVERTED = false;


    public static final double GEARING = 42750/5250;
    public static final double DISTANCE_PER_ROTATION = 1 / (GEARING);

    public static final int DATA_VECTOR_SIZE = 36000;
}
