#!/usr/bin/env python

import rospy
import pylab
import graph_generator

import numpy


def get_mean(path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    for i in range(len(data[0])):
        data_column = [row[i] for row in data]
        print numpy.mean(data_column[700:1000])

        fit = pylab.polyfit(range(len(data_column)), data_column, 2)
        print fit


def get_score_mean(path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line.split(' ')) for line in my_file ]

    for i in range(len(data[0])):
        data_column = [row[i] for row in data]
        print 'score ', numpy.mean(data_column[700:1000])

        fit = pylab.polyfit(range(len(data_column)), data_column, 2)
        print fit


def get_data():
    path = graph_generator.get_path_to_package('log_30_3_behaviors_3_features_original_7/')
    log_files = graph_generator.get_log_files(path)
    for log_file in log_files:
        if log_file.endswith('.txt'):
            if log_file.startswith('match_behaviors_'):
                get_mean(path, log_file)
            if log_file.startswith('match_scores_'):
                get_score_mean(path, log_file)

    
    pass


if __name__ == '__main__':
    try:
        get_data()
    except rospy.ROSInterruptException: pass