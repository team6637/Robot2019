package frc.robot;

public class RobotState {
  
  public enum State {
    HOME,
    CARGO_INTAKE,
    CARGO_1,
    CARGO_2,
    CARGO_3,
    HATCH_1,
    HATCH_2,
    HATCH_3,
  }

  public static State currentState;

  public RobotState() {
    currentState = State.HOME;
  }
}