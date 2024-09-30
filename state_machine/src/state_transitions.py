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

#######################
# PRREDICTIVE SPLINER TRANSITIONS #
#######################
def PSGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    valid_spline = state_machine._check_availability_splini_wpts()
    enemy_in_front = state_machine._check_enemy_in_front()
    ot_sector = state_machine._check_ot_sector()
    gb_free = state_machine._check_gbfree()
    gb_predict_free = state_machine._check_prediction_gbfree()
    o_free = state_machine._check_ofree()
    force_trailing = state_machine._check_force_trailing()

    if not state_machine._check_only_ftg_zone():
        if force_trailing:
            print("Global --> Trailing: force_trailing")
            return StateType.TRAILING
        elif valid_spline and o_free and ot_sector:
            return StateType.OVERTAKE
        elif enemy_in_front and gb_free and gb_predict_free and ot_sector and not force_trailing:
            return StateType.GB_TRACK
        elif enemy_in_front:
            print("Global --> Trailing: enemy_in_front")
            return StateType.TRAILING
        else:
            return StateType.GB_TRACK
    else:
        return StateType.FTGONLY


def PSTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    ot_sector = state_machine._check_ot_sector()
    valid_spline = state_machine._check_availability_splini_wpts()
    emergency_break = state_machine._check_emergency_break()
    enemy_in_front = state_machine._check_enemy_in_front()
    gb_free = state_machine._check_gbfree()
    gb_predict_free = state_machine._check_prediction_gbfree()
    o_free = state_machine._check_ofree()
    on_avoidance_spline = state_machine._check_on_spline()
    on_merger = state_machine._check_on_merger()
    force_trailing = state_machine._check_force_trailing()

    if not state_machine._check_only_ftg_zone():
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_jesus_take_the_wheel():
            return StateType.FTGONLY
        elif force_trailing:
            print("Trailing --> Trailing: force_trailing")
            return StateType.TRAILING
        elif valid_spline and not emergency_break and o_free and ot_sector and not on_merger:
            return StateType.OVERTAKE
        elif not enemy_in_front and on_avoidance_spline and not on_merger: # Questionable if on_merger really helps in this case
            return StateType.OVERTAKE
        elif not enemy_in_front:
            return StateType.GB_TRACK
        elif  gb_free and gb_predict_free and ot_sector: #  enemy_in_front and and best_ot_sector:
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def PSOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone():
        ot_sector = state_machine._check_ot_sector()
        spline_valid = state_machine._check_availability_splini_wpts()
        enemy_in_front = state_machine._check_enemy_in_front()
        emergency_break = state_machine._check_emergency_break()
        on_avoidance_spline = state_machine._check_on_spline()
        o_free = state_machine._check_ofree()
        force_trailing = state_machine._check_force_trailing()

        if emergency_break or not ot_sector:
            print("emergency break or not ot sector")
            return StateType.TRAILING
        elif not o_free or force_trailing:
            print("OT --> Trailing: not o free")
            print("force_trailing:", force_trailing)
            print("o_free:", o_free)
            return StateType.TRAILING
        elif spline_valid and o_free and ot_sector:
            return StateType.OVERTAKE
        elif not spline_valid and not enemy_in_front and on_avoidance_spline:
            return StateType.OVERTAKE
        elif not spline_valid and not enemy_in_front and not on_avoidance_spline:
            return StateType.GB_TRACK
        else:
            print("OT --> Trailing: default trailing")
            print("spline_valid:", spline_valid)
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def PSFTGOnlyTransition(state_machine: StateMachine) -> StateType:
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
