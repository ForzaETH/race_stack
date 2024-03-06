import numpy as np
import csv

from typing import List, Tuple

# Save a list of poses


def save_csv(data_list: List[Tuple], filename: str, header: List[str] = None) -> None:
    '''Saves an arbitrary data file (list of tuples) into a named CSV'''

    if filename[-4:] != ".csv":
        filename += ".csv"

    with open(filename, 'w') as file:
        writer = csv.writer(file)

        if header is not None:
            writer.writerow(header)

        for data in data_list:
            writer.writerow(data)


def read_csv(filename: str, has_header=True) -> List[Tuple]:
    '''Reads a data file (list of tuples) from a named CSV.

    If has_header, then we skip the first row.'''
    out = []

    with open(filename, 'r') as file:
        reader = csv.reader(file)

        if has_header:
            print(f"Opened file {filename} has header:\n", next(reader, None))

        for row in reader:
            elem = tuple(map(float, row))
            out.append(elem)

    return out
