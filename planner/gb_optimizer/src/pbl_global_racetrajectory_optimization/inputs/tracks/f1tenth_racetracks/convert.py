# MIT License

# Copyright (c) 2020 Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.



"""
Conversion from x-axis pointing up (in map img), 0 to pi c.c.w., and 0 to -pi c.w. convention to x-axis pointing right (im map img), 0 to 2pi c.c.w. convention.
Use either on one csv file or all csv file in a directory
Author: Hongrui Zheng
"""

import argparse
import numpy as np
import pandas as pd
import glob
import csv
import os

"""
Script that convert coordinate system conventions

Args:
    --pattern (str): pattern for glob, converts all matching file
"""

parser = argparse.ArgumentParser()
parser.add_argument('--pattern', default='*/*raceline.csv')
args = parser.parse_args()

all_files = glob.glob(args.pattern)
print('Converting following files:')
for name in all_files:
    print(name)
input('Press ENTER to proceed, CTRL+C to stop.')

for file in all_files:
    # get file name and extension
    file_name, file_ext = os.path.splitext(file)

    # create new file name
    new_file = file_name + '_newconv' + file_ext

    print('Working on: ' + file)
    
    # keep original headers
    headers = list(csv.reader(open(file)))[0:3]
    
    # csv to dataframe
    df = pd.read_csv(file, sep=';', header=2)
    
    # converting the headings column
    heading_np = df[' psi_rad'].to_numpy()
    heading_np += np.pi/2
    heading_np[heading_np > 2*np.pi] -= 2*np.pi
    heading_np[heading_np < 0] += 2*np.pi
    df[' psi_rad'].replace(heading_np)

    # save to new file
    f = open(new_file, 'w')
    csv_writer = csv.writer(f)
    csv_writer.writerows(headers)
    f.close()
    df.to_csv(new_file, sep=';', header=False, index=False, float_format='%.7f', mode='a')

    print('New convention saved to: ' + new_file)

print('All files done.')