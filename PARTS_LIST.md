# Parts List

## Subsystems
### Shooter
- 2 NEOs, each powering one side of flywheels
- 1 NEO powering the rotating the shooter assembly
- 1 REV Through Bore Encoder measuring the shooter's angle
  - Absolute encoder
  - Plugged into the pivot SPARK MAX's data port

### Intake
- 1 NEO powering the intake
- A method of determining when we have possession of a NOTE (your choice):
  - Color sensor
  - Limit switch
- 1 NEO powering a set of wheels that exchanges NOTEs between the Intake and the Shooter

### Climber
- 1 NEO powering a winch that retracts/extends the hooked climber

## Notes
- Add variables to `constants.py` for motor IDs, motor inversions, etc. so we can decide on these specifics later.
- Using velocity control to hold the climber in place (velocity = 0) may be necessary, but start with using only percent output control.
- The robot's global position (on the field) and position relative to any AprilTags in sight will be available.
  - Useful for commands that adjust something based on distance to a target (e.g., the SPEAKER)