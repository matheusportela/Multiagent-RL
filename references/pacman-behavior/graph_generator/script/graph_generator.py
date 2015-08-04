#!/usr/bin/env python

import rospy
import matplotlib
import matplotlib.pyplot as plt
import pylab

import rospkg

from os import listdir
from os.path import isfile, join

matplotlib.rcParams['lines.markersize'] = .5*matplotlib.rcParams['lines.markersize']

def get_path_to_package(folder='log_50_5_behaviors_6_features_no_errors_11/'):
    rospack = rospkg.RosPack()
    path = rospack.get_path('graph_generator')
    return path + '/logs/' + folder

def get_log_files(path):
    log_files = [ f for f in listdir(path) if isfile(join(path,f)) ]
    return log_files

def gen_polinomial_function(polinomial_args):
    def polinomy(x_list):
        return_list = []

        for x in x_list:
            result = 0.0
            count = len(polinomial_args)
            for arg in polinomial_args:
                count -= 1
                result += arg * x**count
            return_list.append(result)

        return return_list
    return polinomy

def plot_graph(path, file_name):
    my_file = open ( path + file_name , 'r')
    if file_name.startswith( 'match_scores' ) or file_name.startswith( 'chosen' ) or file_name.startswith( 'match_time' ):
        data = [ map(float, line.split(' ')) for line in my_file ]
    else:
        data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    matplotlib.rcParams['lines.markersize'] = 0.5*matplotlib.rcParams['lines.markersize']

    fig1 = plt.figure(figsize=(8.0, 5.0))
    ax = fig1.add_subplot(111)
    ax.plot(data, '.')
    save_file_name = path + 'image_hd__' + file_name[:-4] + '.png'
    fig1.savefig(save_file_name, dpi=200)

    plt.close(fig1)

    matplotlib.rcParams['lines.markersize'] = 2*matplotlib.rcParams['lines.markersize']

    fig2 = plt.figure(figsize=(8.0, 5.0))
    ax = fig2.add_subplot(111)
    if file_name.startswith( 'behavior' ) or file_name.startswith( 'per_match_behavior' ):
        plots = []

        for i in range(len(data[0])):
            data_column = [row[i] for row in data]

            # ax.plot(range(len(data_column)), data_column, '.', range(len(data_column)), fit_fn(data_column), '--')
            p, = ax.plot(range(len(data_column)), data_column, '.')
            plots.append(p)
            #plt.show()

        ax.legend(plots, ['Bias', 'Dist Comida', 'Dist Capsula', 'Prob Fantasma', ' Prob Branco Fant'])
    else:
        ax.plot(data, '.')

    save_file_name = path + 'image__' + file_name[:-4] + '.png'
    fig2.savefig(save_file_name, dpi=200)

    plt.close(fig2)

    if file_name.startswith( 'match_scores' ):

        matplotlib.rcParams['lines.markersize'] = 0.125*matplotlib.rcParams['lines.markersize']

        for polinomy_counter in range(15):

            fig3 = plt.figure(figsize=(8.0, 5.0))
            ax = fig3.add_subplot(111)
            plots = []

            data_column = [row[0] for row in data]

            fit = pylab.polyfit(range(len(data_column)), data_column, polinomy_counter)
            fit_fn = gen_polinomial_function(fit) # pylab.poly1d(fit)

            # ax.plot(range(len(data_column)), data_column, '.', range(len(data_column)), fit_fn(data_column), '--')
            #p, = ax.plot(data, '.')
            p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), '--')
            #plots.append(p)
            #plt.show()

            # ax.legend(plots, ['Pontuacao da Partida'])
            save_file_name = path + 'image__extra__' + file_name[:-4] +  '____pol' + str(polinomy_counter) + '.png'
            fig3.savefig(save_file_name, dpi=400)

            plt.close(fig3)

        matplotlib.rcParams['lines.markersize'] = 8*matplotlib.rcParams['lines.markersize']

    if file_name.startswith( 'match_behaviors' ):

        matplotlib.rcParams['lines.markersize'] = 0.125*matplotlib.rcParams['lines.markersize']

        for polinomy_counter in range(15):

            fig3 = plt.figure(figsize=(8.0, 5.0))
            ax = fig3.add_subplot(111)
            plots = []

            for i in range(len(data[0])):
                data_column = [row[i] for row in data]

                fit = pylab.polyfit(range(len(data_column)), data_column, polinomy_counter)
                fit_fn = gen_polinomial_function(fit) # pylab.poly1d(fit)

                # ax.plot(range(len(data_column)), data_column, '.', range(len(data_column)), fit_fn(data_column), '--')
                p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), '--')
                plots.append(p)
                #plt.show()

            ax.legend(plots, ['Parar', 'Comer', 'Fugir', 'Comer Capsula', 'Cacar'])
            #save_file_name = path + 'image__extra____pol' + str(polinomy_counter) + '__' + file_name[:-4] + '.png'
            save_file_name = path + 'image__extra__' + file_name[:-4] +  '____pol' + str(polinomy_counter) + '.png'
            fig3.savefig(save_file_name, dpi=400)

            plt.close(fig3)

        matplotlib.rcParams['lines.markersize'] = 8*matplotlib.rcParams['lines.markersize']

def generate_graphs():
    print 'Generating Graphs'
    path = get_path_to_package()
    log_files = get_log_files(path)
    for log_file in log_files:
        if log_file.endswith('.txt'):
            plot_graph(path, log_file)

if __name__ == '__main__':
    try:
        generate_graphs()
    except rospy.ROSInterruptException: pass
    
