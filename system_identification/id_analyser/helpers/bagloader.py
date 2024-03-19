from bagpy import bagreader
import pandas as pd
import numpy as np
import os
import sys

time_fields = ["header.stamp.secs" ,"header.stamp.nsecs", "Time"]

def get_bag_topic(reader, topic):
  try:
    data = reader.message_by_topic(topic)
  except ValueError as error:
    print(error.args)
    print("Likely the topic does not exist or wasn't published to")
    sys.exit(1)
  return data

def get_bag_df(f, field_dict):
  print("Loading Bag " + f + "...")
  reader = bagreader(f)
  bag_df = []
  bag_start = 0
  bag_end = 0
  for topic, fields in field_dict.items():
    print("___ IMPORTING TOPIC " + topic )
    merge_time = 'Time'
    has_stamp = False
    topic_csv = get_bag_topic(reader, topic)
    available_fields = pd.read_csv(topic_csv, nrows=0).columns.tolist()
    # get the fields for that topic:
    if time_fields[0] in available_fields:
      # topic is stamped, use stamp time fields
      try:
        topic_df = pd.read_csv(topic_csv, usecols=fields + time_fields)
        if len(topic_df[(topic_df['header.stamp.nsecs'] != 0.0)]) > 0:
          has_stamp = True
          merge_time = 'header.stamp.nsecs'
      except ValueError as error:
        print(error.args)
        sys.exit(1)
    if has_stamp == False:
      # topic is not stamped
      merge_time = 'Time'
      has_stamp = False
      try:
        topic_df = pd.read_csv(topic_csv, usecols=fields + ['Time'])
      except ValueError as error:
        print(error.args)
        sys.exit(1)

    # handle commands via float64
    topic_df.rename(index=str, columns={"data": topic + ".data"}, inplace=True)

    # handle default ackermann drive command
    if "drive.speed" in available_fields:
      topic_df.drop(topic_df[(topic_df['drive.speed'] == 0.0) & (topic_df['drive.acceleration'] == 0.0)].index, inplace=True, axis=0)

    if has_stamp:
      # merge_time = 'header.stamp.nsecs'
      # combine secs and n_secs into one column used for merging
      topic_df.drop(topic_df[(topic_df['header.stamp.nsecs'] == 0.0)].index, inplace=True, axis=0)
      topic_df['header.stamp.nsecs'] = topic_df['header.stamp.secs']*pow(10,9) + topic_df['header.stamp.nsecs']
      topic_df.drop('header.stamp.secs', inplace=True, axis=1)
    topic_start = topic_df[merge_time].iloc[0]
    topic_end = topic_df[merge_time].iloc[-1]
    
    # first one does not need to be merged, merge subsequent ones on timestamp
    if len(bag_df) == 0:
      bag_df = topic_df
      bag_start = topic_start
      bag_end = topic_end
    else:
      #if the current topic was published later than the bag_df so far, crop the bag_df, otherwise the topic_df
      if (topic_start > bag_start):
        bag_start = topic_start
      else:
        topic_start = bag_start
      if (bag_end > topic_end):
        bag_end = topic_end
      else:
        topic_end = bag_end

      bag_df = bag_df[(bag_df[merge_time] >= bag_start) | (bag_df[merge_time] <= bag_end)]
      topic_df = topic_df[(topic_df[merge_time] >= topic_start) | (topic_df[merge_time] <= topic_end)]

      # check which one is shorter and select that one as left merge item
      # merge finds a match for everything on the left and drops anything that's not matched
      if bag_df.size < topic_df.size:
        # if we are merging on header.nsecs, keep only one Time column
        if has_stamp:
          topic_df.drop('Time', inplace=True, axis=1)
        try:
          bag_df = pd.merge_asof(bag_df, topic_df, on=[merge_time])
        except pd.errors.MergeError as error:
          print("bag df was: \n")
          print(bag_df.head())
          print("topic df was: \n")
          print(topic_df.head())
          print("Tried to merge on " + merge_time + " with bag df on the left")
          print(error.args)
          sys.exit(1)
      else:
        if has_stamp:
          bag_df.drop('Time', inplace=True, axis=1)
        try:
          bag_df = pd.merge_asof(topic_df, bag_df, on=[merge_time])
        except pd.errors.MergeError as error:
          print("bag df was: \n")
          print(bag_df.head())
          print("topic df was: \n")
          print(topic_df.head())
          print("Tried to merge on " + merge_time + " with topic df on the left")
          print(error.args)
          sys.exit(1)
      # drop nan's
      bag_df.dropna(inplace=True)
      bag_df = bag_df.astype({'header.stamp.nsecs':'int64'})
  return bag_df

def load_bags(directory, field_dict):
  # iterate over files in directory
  dataframe_list = [] # array to store dataframes that need to be merged later on
  if not os.path.exists(directory):
    raise ValueError(f"Directory {directory} does not exist")
  if os.listdir(directory) == []:
    raise ValueError(f"Directory {directory} is empty")
  for filename in os.listdir(directory):
    f = os.path.join(directory, filename)
    # checking if it is a file
    if os.path.splitext(f)[1] == ".bag":
      dataframe_list.append(get_bag_df(f, field_dict))
  # concatenate all the dataframes from the bags:
  bags_df = pd.concat(dataframe_list, ignore_index=True)
  print("Total number of datapoints: " + str(bags_df.shape[0]))
  return bags_df
