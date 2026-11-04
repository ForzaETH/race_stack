import argparse
import numpy as np
import pandas as pd
import dataclasses
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys import get_types_from_msg

typestore = get_typestore(Stores.ROS2_HUMBLE)

msg_dir = Path(get_package_share_directory('f110_msgs')) / 'msg'

add_types = {}
for msg_file in msg_dir.glob('*.msg'):
    text = msg_file.read_text()
    for pkg in ('rs_msgs', 'f110_msgs'):
        add_types.update(get_types_from_msg(text, f'{pkg}/msg/{msg_file.stem}'))
typestore.register(add_types)


def flatten(msg, prefix=''):
    """Macht aus einer verschachtelten Nachricht ein flaches Dict."""
    result = {}
    for f in dataclasses.fields(msg):
        value = getattr(msg, f.name)
        key = f'{prefix}{f.name}'
        if dataclasses.is_dataclass(value):
            result.update(flatten(value, key + '.'))
        elif isinstance(value, np.ndarray):
            # Arrays als Liste speichern (z. B. covariance), sonst fehlen sie
            result[key] = value.tolist()
        elif isinstance(value, (list, tuple)):
            # Liste von Unter-Nachrichten (z. B. markers) nur als Anzahl
            if value and dataclasses.is_dataclass(value[0]):
                result[key + '.count'] = len(value)
            else:
                result[key] = list(value)
        else:
            result[key] = value
    return result


def main():
    # parse_known_args, because launch appends --ros-args to the command line
    parser = argparse.ArgumentParser()
    parser.add_argument('-input', required=True, type=Path, help='Bag folder to convert')
    input_bag = parser.parse_known_args()[0].input

    # results/output-1 is written to results/converted-1, next to the input bag
    out_dir = input_bag.parent / f'converted-{input_bag.name.removeprefix("output-")}'
    print(input_bag)
    print(out_dir)

    with AnyReader([input_bag], default_typestore=typestore) as reader:
        out_dir.mkdir(parents=True, exist_ok=True)
        for conn in reader.connections:

            name = conn.topic.strip('/').replace('/', '_')
            print(f'{conn.topic} ({conn.msgtype}): {conn.msgcount} Nachrichten')

            if conn.msgtype not in typestore.fielddefs:
                print('  -> übersprungen (Typ unbekannt)')
                continue

            if conn.msgtype in ('sensor_msgs/msg/Image',
                                'sensor_msgs/msg/PointCloud2',
                                'sensor_msgs/msg/CompressedImage'):
                print('  -> übersprungen (zu groß, extra behandeln)')
                continue

            if conn.msgtype == 'sensor_msgs/msg/LaserScan':
                times, ranges = [], []
                for c, t, raw in reader.messages(connections=[conn]):
                    msg = reader.deserialize(raw, c.msgtype)
                    times.append(t * 1e-9)
                    ranges.append(msg.ranges)
                np.savez(out_dir / f'{name}.npz',
                         time=np.array(times), ranges=np.array(ranges))
                print(f'  -> {name}.npz')
                continue

            rows = []
            for c, t, raw in reader.messages(connections=[conn]):
                msg = reader.deserialize(raw, c.msgtype)
                if conn.msgtype == 'tf2_msgs/msg/TFMessage':
                    for tr in msg.transforms:
                        row = {'time_s': t * 1e-9}
                        row.update(flatten(tr))
                        rows.append(row)
                else:
                    row = {'time_s': t * 1e-9}
                    row.update(flatten(msg))
                    rows.append(row)

            pd.DataFrame(rows).to_csv(out_dir / f'{name}.csv', index=False)
            print(f'  -> {name}.csv')


if __name__ == '__main__':
    main()
