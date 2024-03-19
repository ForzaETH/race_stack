from __future__ import annotations

from typing import TYPE_CHECKING

from states_types import StateType

if TYPE_CHECKING:
    from state_machine_node import StateMachine

"""
Transitions should loosely follow the following template (basically a match-case)

if (logic sum of bools obtained by methods of state_machine):   
    return StateType.<DESIRED STATE>
elif (e.g. state_machine.obstacles are near):
    return StateType.<ANOTHER DESIRED STATE>
...

NOTE: ideally put the most common cases on top of the match-case

NOTE 2: notice that, when implementing new states, if an attribute/condition in the 
    StateMachine is not available, your IDE will tell you, but only if you have a smart 
    enough IDE. So use vscode, pycharm, fleet or whatever has specific python syntax highlights.

NOTE 3: transistions must not have side effects on the state machine! 
    i.e. any attribute of the state machine should not be modified in the transitions.

"""


#######################
# SPLINER TRANSITIONS #
#######################
def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone():
        if state_machine._check_gbfree():
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree()
    ot_sector = state_machine._check_ot_sector()

    if not state_machine._check_only_ftg_zone():
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg():
            return StateType.FTGONLY
        if not gb_free and not ot_sector:
            return StateType.TRAILING
        elif gb_free and state_machine._check_close_to_raceline():
            return StateType.GB_TRACK
        elif (
            not gb_free
            and ot_sector
            and state_machine._check_availability_splini_wpts()
            and state_machine._check_ofree()
        ):
            return StateType.OVERTAKE
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone():
        in_ot_sector = state_machine._check_ot_sector()
        spline_valid = state_machine._check_availability_splini_wpts()
        o_free = state_machine._check_ofree()

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING
        if in_ot_sector and o_free and spline_valid:
            return StateType.OVERTAKE
        # If spline becomes unvalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree():
            return StateType.GB_TRACK
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone():
        return StateType.FTGONLY
    else:
        if state_machine._check_close_to_raceline() and state_machine._check_gbfree():
            return StateType.GB_TRACK
        else:
            return StateType.FTGONLY


####################################
# OTHER TRANSITIONS  could go here #
####################################
# def SomeOtherStateTransition(state_machine: StateMachine) -> StateType:
