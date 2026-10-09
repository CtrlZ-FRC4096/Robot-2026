"""The robot's state machine: what the intake, shooter and drivetrain are doing.

The robot is always in exactly one IntakeState, one ShooterState and one
DriveState. Buttons, autonomous steps and the shooter itself send Events, and
the three transition tables below are the complete list of state changes:
nothing else assigns a state. An event that a table does not list for the
current state leaves that part of the robot where it is.

Subsystems only read the states (and the few derived questions at the bottom
of RobotStateMachine) to decide what their motors do each loop.

Run `python state_machine.py` to print the tables as a Mermaid diagram.
"""

from enum import Enum


class IntakeState(Enum):
    STOWED = "Stowed"              # pivot up, rollers off
    DEPLOYED = "Deployed"          # pivot down, rollers off; agitates while the shooter feeds
    INTAKING = "Intaking"          # pivot down, rollers pulling fuel in
    CLEARING_JAM = "Clearing Jam"  # pivot down, rollers reversed


class ShooterState(Enum):
    STOWED = "Stowed"                            # flywheels, accelerator and indexer off, hood down
    AIMING = "Aiming"                            # drivetrain turns to the target; shooter still off
    SPINNING_UP = "Spinning Up"                  # aim; flywheels and hood from the distance lookup
    FEEDING = "Feeding"                          # ... and the indexer feeds fuel
    MANUAL_SPINNING_UP = "Manual Spinning Up"    # fixed "down bad" speed and hood, no aiming
    MANUAL_FEEDING = "Manual Feeding"            # ... and the indexer feeds fuel
    SPINNING_DOWN = "Spinning Down"              # still aiming; indexer and accelerator stop, then flywheels
    CLEARING_JAM = "Clearing Jam"                # flywheels, accelerator and indexer reversed


class DriveState(Enum):
    DRIVER_CONTROL = "Driver Control"  # joysticks in teleop, PathPlanner/aiming in auto
    LINEUP = "Lineup"                  # PID to lineup_pose (heading to the target while aiming)


class Event(Enum):
    # Driver 1
    INTAKE_PRESSED = "intake pressed"            # left trigger
    INTAKE_RELEASED = "intake released"
    SHOOT_PRESSED = "shoot pressed"              # right trigger
    SHOOT_RELEASED = "shoot released"
    STOW_INTAKE = "stow intake"                  # POV left: stow the intake, cancel an aimed shot
    STOW_ALL = "stow all"                        # left bumper
    LINEUP_PRESSED = "lineup pressed"            # right bumper: drive to the trench
    LINEUP_RELEASED = "lineup released"
    # Driver 2
    MANUAL_SHOT_PRESSED = "manual shot pressed"  # POV right ("down bad")
    MANUAL_SHOT_RELEASED = "manual shot released"
    CLEAR_JAM_PRESSED = "clear jam pressed"      # POV left
    CLEAR_JAM_RELEASED = "clear jam released"
    # Autonomous steps (coroutines.py)
    AUTO_INTAKE = "auto intake"
    AUTO_INTAKE_AND_SPIN_DOWN = "auto intake and spin down"
    AUTO_STOP_SHOOTING = "auto stop shooting"
    AUTO_SHOOT = "auto shoot"
    AUTO_LINEUP_STARTED = "auto lineup started"
    AUTO_LINEUP_FINISHED = "auto lineup finished"
    # Match phases
    AUTONOMOUS_STARTED = "autonomous started"
    TELEOP_STARTED = "teleop started"
    # Raised by the shooter from its sensors
    FEEDING_STARTED = "feeding started"          # accelerator up to speed
    SPIN_DOWN_FINISHED = "spin down finished"    # indexer and accelerator stopped


ANY = "any state"  # wildcard in a transition table row

_I, _S, _D, _E = IntakeState, ShooterState, DriveState, Event

# Event -> {from state: to state}. ANY matches every state not listed.
INTAKE_TRANSITIONS = {
    _E.INTAKE_PRESSED:            {_I.STOWED: _I.INTAKING, _I.DEPLOYED: _I.INTAKING},
    _E.INTAKE_RELEASED:           {_I.INTAKING: _I.DEPLOYED},
    _E.SHOOT_RELEASED:            {_I.STOWED: _I.DEPLOYED},
    _E.STOW_INTAKE:               {ANY: _I.STOWED},
    _E.STOW_ALL:                  {ANY: _I.STOWED},
    _E.CLEAR_JAM_PRESSED:         {ANY: _I.CLEARING_JAM},
    _E.CLEAR_JAM_RELEASED:        {ANY: _I.DEPLOYED},
    _E.AUTO_INTAKE:               {_I.CLEARING_JAM: _I.CLEARING_JAM, ANY: _I.INTAKING},
    _E.AUTO_INTAKE_AND_SPIN_DOWN: {_I.CLEARING_JAM: _I.CLEARING_JAM, ANY: _I.INTAKING},
    _E.AUTO_STOP_SHOOTING:        {_I.CLEARING_JAM: _I.CLEARING_JAM, ANY: _I.DEPLOYED},
    _E.AUTO_SHOOT:                {_I.CLEARING_JAM: _I.CLEARING_JAM, ANY: _I.DEPLOYED},
    _E.AUTO_LINEUP_STARTED:       {_I.STOWED: _I.DEPLOYED},
    _E.AUTO_LINEUP_FINISHED:      {_I.INTAKING: _I.DEPLOYED},
    _E.AUTONOMOUS_STARTED:        {_I.STOWED: _I.DEPLOYED},
}

SHOOTER_TRANSITIONS = {
    _E.SHOOT_PRESSED: {_S.STOWED: _S.SPINNING_UP, _S.AIMING: _S.SPINNING_UP,
                       _S.SPINNING_DOWN: _S.SPINNING_UP},
    _E.SHOOT_RELEASED: {_S.CLEARING_JAM: _S.CLEARING_JAM, ANY: _S.SPINNING_DOWN},
    _E.STOW_INTAKE: {_S.AIMING: _S.STOWED, _S.SPINNING_UP: _S.STOWED, _S.FEEDING: _S.STOWED,
                     _S.SPINNING_DOWN: _S.STOWED},
    _E.STOW_ALL: {ANY: _S.STOWED},
    _E.MANUAL_SHOT_PRESSED: {_S.FEEDING: _S.MANUAL_FEEDING, _S.MANUAL_FEEDING: _S.MANUAL_FEEDING,
                             _S.CLEARING_JAM: _S.CLEARING_JAM, ANY: _S.MANUAL_SPINNING_UP},
    _E.MANUAL_SHOT_RELEASED: {_S.CLEARING_JAM: _S.CLEARING_JAM, ANY: _S.SPINNING_DOWN},
    _E.CLEAR_JAM_PRESSED: {ANY: _S.CLEARING_JAM},
    _E.CLEAR_JAM_RELEASED: {_S.SPINNING_DOWN: _S.SPINNING_DOWN, ANY: _S.STOWED},
    _E.AUTO_INTAKE: {ANY: _S.STOWED},
    _E.AUTO_INTAKE_AND_SPIN_DOWN: {_S.CLEARING_JAM: _S.CLEARING_JAM, ANY: _S.SPINNING_DOWN},
    _E.AUTO_STOP_SHOOTING: {_S.CLEARING_JAM: _S.CLEARING_JAM, ANY: _S.SPINNING_DOWN},
    _E.AUTO_SHOOT: {_S.STOWED: _S.SPINNING_UP, _S.AIMING: _S.SPINNING_UP,
                    _S.SPINNING_DOWN: _S.SPINNING_UP},
    _E.AUTO_LINEUP_FINISHED: {_S.STOWED: _S.AIMING},
    _E.TELEOP_STARTED: {ANY: _S.STOWED},
    _E.FEEDING_STARTED: {_S.SPINNING_UP: _S.FEEDING, _S.MANUAL_SPINNING_UP: _S.MANUAL_FEEDING},
    _E.SPIN_DOWN_FINISHED: {_S.SPINNING_DOWN: _S.STOWED},
}

DRIVE_TRANSITIONS = {
    _E.LINEUP_PRESSED: {ANY: _D.LINEUP},
    _E.AUTO_LINEUP_STARTED: {ANY: _D.LINEUP},
    _E.LINEUP_RELEASED: {ANY: _D.DRIVER_CONTROL},
    _E.AUTO_LINEUP_FINISHED: {ANY: _D.DRIVER_CONTROL},
    _E.AUTO_INTAKE: {ANY: _D.DRIVER_CONTROL},
    _E.AUTO_INTAKE_AND_SPIN_DOWN: {ANY: _D.DRIVER_CONTROL},
    _E.AUTO_STOP_SHOOTING: {ANY: _D.DRIVER_CONTROL},
    _E.TELEOP_STARTED: {ANY: _D.DRIVER_CONTROL},
}


def next_state(table, event, state):
    row = table.get(event, {})
    return row.get(state, row.get(ANY, state))


class RobotStateMachine:
    """Current states plus the lineup target; change them only with handle()."""

    def __init__(self):
        self.intake = IntakeState.STOWED
        self.shooter = ShooterState.STOWED
        self.drive = DriveState.DRIVER_CONTROL
        # Lineup data, meaningful while drive is LINEUP.
        self.lineup_pose = None
        self.lineup_speed_limited = False
        self.lineup_arrived = False
        self.last_event = None

    def handle(self, event):
        self.intake = next_state(INTAKE_TRANSITIONS, event, self.intake)
        self.shooter = next_state(SHOOTER_TRANSITIONS, event, self.shooter)
        self.drive = next_state(DRIVE_TRANSITIONS, event, self.drive)
        self.last_event = event

    def start_lineup(self, event, pose, speed_limited=False):
        """Send LINEUP_PRESSED or AUTO_LINEUP_STARTED with its target pose."""
        self.lineup_pose = pose
        self.lineup_speed_limited = speed_limited
        self.lineup_arrived = False
        self.handle(event)

    # Questions the subsystems ask about the current states.

    @property
    def aiming(self):
        """The drivetrain points the shooter at the target and shot values are computed."""
        return self.shooter in (ShooterState.AIMING, ShooterState.SPINNING_UP,
                                ShooterState.FEEDING, ShooterState.SPINNING_DOWN)

    @property
    def shooting(self):
        """Flywheels and hood track a shot (aimed or manual)."""
        return self.shooter in (ShooterState.SPINNING_UP, ShooterState.FEEDING,
                                ShooterState.MANUAL_SPINNING_UP, ShooterState.MANUAL_FEEDING)

    @property
    def manual_shot(self):
        return self.shooter in (ShooterState.MANUAL_SPINNING_UP, ShooterState.MANUAL_FEEDING)

    @property
    def feeding(self):
        return self.shooter in (ShooterState.FEEDING, ShooterState.MANUAL_FEEDING)

    @property
    def lining_up(self):
        return self.drive is DriveState.LINEUP


def mermaid():
    """The transition tables as Mermaid state diagrams (one per part)."""
    lines = []
    for title, table, states in (("Intake", INTAKE_TRANSITIONS, IntakeState),
                                 ("Shooter", SHOOTER_TRANSITIONS, ShooterState),
                                 ("Drive", DRIVE_TRANSITIONS, DriveState)):
        lines += [f"%% {title}", "stateDiagram-v2"]
        edges = {}
        for event, row in table.items():
            for state in states:
                target = next_state(table, event, state)
                if target is not state and (state in row or ANY in row):
                    edges.setdefault((state.name, target.name), []).append(event.value)
        for (src, dst), events in edges.items():
            lines.append(f"    {src} --> {dst}: {', '.join(events)}")
        lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    print(mermaid())
