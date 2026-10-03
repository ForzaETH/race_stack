import difflib
import os
import sys

from launch.action import Action
from launch.frontend import expose_action
from launch.launch_description_sources import AnyLaunchDescriptionSource

# appended by `ros2 launch` itself when --launch-prefix(-filter) is used
_ROS2_LAUNCH_ARGS = {'launch-prefix', 'launch-prefix-filter'}

_RED = '\033[31m'
_RESET = '\033[0m'


@expose_action('check_args')
class CheckArgs(Action):
    """
    Fail the launch if a `name:=value` argument is not declared in this launch file itself.
    """

    @classmethod
    def parse(cls, entity, parser):
        _, kwargs = super().parse(entity, parser)
        return cls, kwargs

    def execute(self, context):
        launch_file = context.get_locals_as_dict().get('current_launch_file_path')
        if launch_file is None or os.path.basename(launch_file) not in map(os.path.basename, sys.argv):
            return None

        given = [arg.split(':=', 1)[0] for arg in context.argv if ':=' in arg]
        description = AnyLaunchDescriptionSource(launch_file).try_get_launch_description_without_context()
        # only this file's own args, not those of its includes (no include chain == declared at top level)
        declared = {
            arg.name
            for arg, include_chain in description.get_launch_arguments_with_include_launch_description_actions()
            if not include_chain
        } | _ROS2_LAUNCH_ARGS

        unknown = [name for name in given if name not in declared]
        if unknown:
            hints = []
            for name in unknown:
                close = difflib.get_close_matches(name, declared, n=1)
                hints.append(f"'{name}'" + (f" (did you mean '{close[0]}'?)" if close else ''))
            message = (
                f"Unknown launch argument(s) for {os.path.basename(launch_file)}: {', '.join(hints)}. "
                f"Valid arguments: {sorted(declared - _ROS2_LAUNCH_ARGS)}"
            )
            # red in a terminal, plain when piped to a file/log
            if sys.stdout.isatty():
                message = f'{_RED}{message}{_RESET}'
            raise RuntimeError(message)
        return None
