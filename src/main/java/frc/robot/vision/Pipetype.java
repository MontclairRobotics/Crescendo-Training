package frc.robot.vision;

public enum Pipetype {
    DRIVER(0),
    NOTE(1);

    protected final int pipe;

    private Pipetype(int pipe) {
        this.pipe = pipe;
    }
    public int getPipe(){
        return pipe;
    }
}
