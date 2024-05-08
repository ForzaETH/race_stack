from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
        
def timetrials_transition(state_machine: StateMachine)->str:
    return StateType.GB_TRACK

def head_to_head_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            return SpliniGlobalTrackingTransition(state_machine)
        case StateType.TRAILING:
            return SpliniTrailingTransition(state_machine)
        case StateType.OVERTAKE:
            return SpliniOvertakingTransition(state_machine)
        case StateType.FTGONLY:
            return SpliniFTGOnlyTransition(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree
    ot_sector = state_machine._check_ot_sector

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            return StateType.TRAILING
        elif gb_free and state_machine._check_close_to_raceline:
            return StateType.GB_TRACK
        elif (
            not gb_free
            and ot_sector
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            return StateType.OVERTAKE
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone:
        in_ot_sector = state_machine._check_ot_sector
        spline_valid = state_machine._check_availability_splini_wpts
        o_free = state_machine._check_ofree

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING
        if in_ot_sector and o_free and spline_valid:
            return StateType.OVERTAKE
        # If spline becomes unvalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree:
            return StateType.GB_TRACK
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline and state_machine._check_gbfree:
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY
        