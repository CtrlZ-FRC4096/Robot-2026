"""Transition-table checks for state_machine.py (no robot or simulation)."""

from collections import deque

import pytest

from state_machine import (ANY, DRIVE_TRANSITIONS, INTAKE_TRANSITIONS, SHOOTER_TRANSITIONS,
                           DriveState as D, Event as E, IntakeState as I, RobotStateMachine,
                           ShooterState as S, mermaid)

TABLES = ((INTAKE_TRANSITIONS, I), (SHOOTER_TRANSITIONS, S), (DRIVE_TRANSITIONS, D))


def run(*events, machine=None):
    machine = machine or RobotStateMachine()
    for event in events:
        machine.handle(event)
    return machine


def test_tables_only_name_their_own_states_and_every_event_is_used():
    used = set()
    for table, states in TABLES:
        for event, row in table.items():
            assert isinstance(event, E)
            used.add(event)
            for source, target in row.items():
                assert source is ANY or isinstance(source, states)
                assert isinstance(target, states)
    assert used == set(E)


def test_every_state_is_reachable_from_power_on():
    start = RobotStateMachine()
    seen = {(start.intake, start.shooter, start.drive)}
    queue = deque([seen.copy().pop()])
    while queue:
        intake, shooter, drive = queue.popleft()
        for event in E:
            machine = RobotStateMachine()
            machine.intake, machine.shooter, machine.drive = intake, shooter, drive
            machine.handle(event)
            key = (machine.intake, machine.shooter, machine.drive)
            if key not in seen:
                seen.add(key)
                queue.append(key)
    assert {k[0] for k in seen} == set(I)
    assert {k[1] for k in seen} == set(S)
    assert {k[2] for k in seen} == set(D)


def test_power_on_and_unlisted_events_leave_state_alone():
    machine = RobotStateMachine()
    assert (machine.intake, machine.shooter, machine.drive) == (I.STOWED, S.STOWED, D.DRIVER_CONTROL)
    run(E.INTAKE_RELEASED, E.FEEDING_STARTED, E.SPIN_DOWN_FINISHED, E.LINEUP_RELEASED, machine=machine)
    assert (machine.intake, machine.shooter, machine.drive) == (I.STOWED, S.STOWED, D.DRIVER_CONTROL)


def test_teleop_shot_cycle():
    machine = run(E.TELEOP_STARTED, E.SHOOT_PRESSED)
    assert machine.shooter is S.SPINNING_UP and machine.aiming and not machine.feeding
    run(E.FEEDING_STARTED, machine=machine)
    assert machine.shooter is S.FEEDING and machine.feeding
    run(E.SHOOT_RELEASED, machine=machine)
    # Releasing the trigger also deploys a stowed intake (as before).
    assert (machine.intake, machine.shooter) == (I.DEPLOYED, S.SPINNING_DOWN)
    assert machine.aiming
    run(E.SPIN_DOWN_FINISHED, machine=machine)
    assert machine.shooter is S.STOWED and not machine.aiming


def test_intaking_is_independent_of_shooting():
    machine = run(E.INTAKE_PRESSED, E.SHOOT_PRESSED, E.FEEDING_STARTED)
    assert (machine.intake, machine.shooter) == (I.INTAKING, S.FEEDING)
    run(E.INTAKE_RELEASED, machine=machine)
    # Deployed while feeding: the intake agitates (see Intake.periodic).
    assert machine.intake is I.DEPLOYED and machine.feeding


def test_manual_shot_does_not_aim_and_spins_down_on_release():
    machine = run(E.MANUAL_SHOT_PRESSED)
    assert machine.shooter is S.MANUAL_SPINNING_UP and machine.manual_shot and not machine.aiming
    run(E.FEEDING_STARTED, machine=machine)
    assert machine.shooter is S.MANUAL_FEEDING and machine.feeding
    run(E.MANUAL_SHOT_RELEASED, machine=machine)
    assert machine.shooter is S.SPINNING_DOWN


def test_clear_jam_takes_over_both_mechanisms_and_releases_to_deployed():
    machine = run(E.INTAKE_PRESSED, E.SHOOT_PRESSED, E.CLEAR_JAM_PRESSED)
    assert (machine.intake, machine.shooter) == (I.CLEARING_JAM, S.CLEARING_JAM)
    run(E.SHOOT_RELEASED, E.INTAKE_PRESSED, E.MANUAL_SHOT_PRESSED, machine=machine)
    assert (machine.intake, machine.shooter) == (I.CLEARING_JAM, S.CLEARING_JAM)
    run(E.CLEAR_JAM_RELEASED, machine=machine)
    assert (machine.intake, machine.shooter) == (I.DEPLOYED, S.STOWED)


def test_stow_buttons():
    machine = run(E.INTAKE_PRESSED, E.SHOOT_PRESSED, E.FEEDING_STARTED, E.STOW_INTAKE)
    # Stow intake also cancels an aimed shot without spinning down.
    assert (machine.intake, machine.shooter) == (I.STOWED, S.STOWED)
    machine = run(E.MANUAL_SHOT_PRESSED, E.STOW_INTAKE)
    assert machine.shooter is S.MANUAL_SPINNING_UP  # a manual shot keeps going
    run(E.STOW_ALL, machine=machine)
    assert (machine.intake, machine.shooter) == (I.STOWED, S.STOWED)


def test_trench_lineup_carries_its_target():
    machine = RobotStateMachine()
    machine.lineup_arrived = True
    machine.start_lineup(E.LINEUP_PRESSED, "trench pose")
    assert machine.lining_up and machine.lineup_pose == "trench pose"
    assert not machine.lineup_arrived and not machine.lineup_speed_limited
    run(E.LINEUP_RELEASED, machine=machine)
    assert machine.drive is D.DRIVER_CONTROL


@pytest.mark.parametrize("second_lineup", [True, False])
def test_autonomous_step_sequence(second_lineup):
    """intake -> lineup over the bump -> shoot -> intake while spinning down ..."""
    machine = run(E.AUTONOMOUS_STARTED)
    assert machine.intake is I.DEPLOYED
    run(E.AUTO_INTAKE, machine=machine)
    assert (machine.intake, machine.shooter) == (I.INTAKING, S.STOWED)
    machine.start_lineup(E.AUTO_LINEUP_STARTED, "bump")
    assert machine.lining_up and machine.intake is I.INTAKING
    run(E.AUTO_LINEUP_FINISHED, machine=machine)
    # One loop of turning toward the hub before the shoot step starts.
    assert (machine.intake, machine.shooter, machine.drive) == (I.DEPLOYED, S.AIMING, D.DRIVER_CONTROL)
    assert machine.aiming
    run(E.AUTO_SHOOT, E.FEEDING_STARTED, machine=machine)
    assert machine.shooter is S.FEEDING
    run(E.AUTO_INTAKE_AND_SPIN_DOWN, machine=machine)
    assert (machine.intake, machine.shooter) == (I.INTAKING, S.SPINNING_DOWN)
    if second_lineup:
        run(E.SPIN_DOWN_FINISHED, machine=machine)
        machine.start_lineup(E.AUTO_LINEUP_STARTED, "bump 2")
        run(E.AUTO_LINEUP_FINISHED, E.AUTO_SHOOT, machine=machine)
    else:
        # Still spinning down when the next shot is requested: shoot anyway
        # (the flag version finished the spin-down and skipped the volley).
        run(E.AUTO_SHOOT, machine=machine)
    assert machine.shooter is S.SPINNING_UP
    run(E.TELEOP_STARTED, machine=machine)
    assert (machine.shooter, machine.drive) == (S.STOWED, D.DRIVER_CONTROL)


def test_cancelling_with_stow_all_then_releasing_does_not_lock_aim():
    # The flag version left the drivetrain locked onto the target after
    # LB + trigger release, ignoring the rotation stick.
    machine = run(E.SHOOT_PRESSED, E.STOW_ALL, E.SHOOT_RELEASED, E.SPIN_DOWN_FINISHED)
    assert machine.shooter is S.STOWED and not machine.aiming


def test_mermaid_lists_every_transition():
    diagram = mermaid()
    assert diagram.count("stateDiagram-v2") == 3
    assert "SPINNING_UP --> FEEDING: feeding started" in diagram
