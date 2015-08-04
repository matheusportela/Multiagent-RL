#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Using the magic encoding


import rospy
import pylab
import graph_generator

import numpy
import matplotlib
import matplotlib.pyplot as plt

import os

num_partidas = 1000
original_map = True
behaviors_3 = False
        


def get_score_mean(path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line.split(' ')) for line in my_file ]

    for i in range(len(data[0])):
        data_column = [row[i] for row in data]
        #print 'score ', numpy.mean(data_column[2700:])

        fit = pylab.polyfit(range(len(data_column)), data_column, 2)
        print fit


def create_score_graph(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line.split(' ')) for line in my_file ]

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    # plots = []

    matplotlib.rcParams['lines.markersize'] = 0.5*matplotlib.rcParams['lines.markersize']
    matplotlib.rcParams['lines.linewidth'] = 2*matplotlib.rcParams['lines.linewidth']

    for i in range(len(data[0])):
        data_column = [row[i] for row in data[:num_partidas]]
        fit = pylab.polyfit(range(len(data_column)), data_column, 4)
        

        fit_fn = graph_generator.gen_polinomial_function(fit) # pylab.poly1d(fit)

        # ax.plot(range(len(data_column)), data_column, '.', range(len(data_column)), fit_fn(data_column), '--')
        p, = ax.plot(data[:num_partidas], 'b.')
        p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), 'r--')
        #plots.append(p)
        #plt.show()

        # ax.legend(plots, ['Pontuacao da Partida'])
        figure.suptitle(u'Pontuação da Partida')
        ax.set_xlabel('Partida')
        ax.set_ylabel(u"Pontuação")
        save_file_name = save_path + 'match_scores____pol.eps'
        figure.savefig(save_file_name, dpi=400)

        plt.close(figure)

    matplotlib.rcParams['lines.markersize'] = 2*matplotlib.rcParams['lines.markersize']
    matplotlib.rcParams['lines.linewidth'] = 0.5*matplotlib.rcParams['lines.linewidth']


def create_chosen_behaviors_graph(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    if behaviors_3:
        behaviors = ['Ficar parado', 'Comer', 'Fugir']
    else:
        behaviors = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Caçar']

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    ax.plot(data, '.')

    matplotlib.rcParams['lines.markersize'] = 0.25*matplotlib.rcParams['lines.markersize']

    if original_map and behaviors_3:
        ax.axis([0, num_partidas, 0, 800])
        pass
    elif ( not original_map ) and behaviors_3:
        ax.axis([0, num_partidas, 0, 550])
    elif original_map and ( not behaviors_3 ):
        ax.axis([0, num_partidas, 0, 800])
    elif ( not original_map ) and ( not behaviors_3 ):
        ax.axis([0, num_partidas, 0, 330])
        
    if not behaviors_3:
        ax.legend(behaviors, loc=2)
    else:
        ax.legend(behaviors)

    figure.suptitle(u'Comportamentos escolhidos')
    ax.set_xlabel('Partida')
    ax.set_ylabel(u"Número de vezes escolhido")
    save_file_name = save_path + 'chosen_behaviors.eps'
    figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.markersize'] = 4*matplotlib.rcParams['lines.markersize']


def create_chosen_behaviors_pol_graph(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    if behaviors_3:
        behaviors = ['Ficar parado', 'Comer', 'Fugir']
    else:
        behaviors = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Caçar']

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    # p, = ax.plot(data, '.')
    plots = []

    matplotlib.rcParams['lines.linewidth'] = 2*matplotlib.rcParams['lines.linewidth']

    for i in range(len(data[0])):
        data_column = [row[i] for row in data]
        fit = pylab.polyfit(range(len(data_column)), data_column, 4)
        fit_fn = graph_generator.gen_polinomial_function(fit)

        p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), '-')
        plots.append(p)
        #plt.show()

    if original_map and behaviors_3:
        ax.axis([0, num_partidas, -20, 250])
        pass
    elif ( not original_map ) and behaviors_3:
        ax.axis([0, num_partidas, -10, 100])
    elif original_map and ( not behaviors_3 ):
        ax.axis([0, num_partidas, -20, 200])
    elif ( not original_map ) and ( not behaviors_3 ):
        ax.axis([0, num_partidas, -10, 80])
        
    if not behaviors_3:
        ax.legend(behaviors, loc=2)
    else:
        ax.legend(behaviors)
    
    figure.suptitle(u'Comportamentos escolhidos')
    ax.set_xlabel('Partida')
    ax.set_ylabel(u"Número de vezes escolhido")
    save_file_name = save_path + 'chosen_behaviors_pol.eps'
    figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.linewidth'] = 0.5*matplotlib.rcParams['lines.linewidth']


def create_chosen_behaviors_pol_graph2(save_path, path, file_name):
    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    # p, = ax.plot(data, '.')
    if len(data[0]) == 3:
        behaviors = ['Ficar parado', 'Comer', 'Fugir']
    else:
        behaviors = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Caçar']

    matplotlib.rcParams['lines.linewidth'] = 2*matplotlib.rcParams['lines.linewidth']

    for i in range(len(data[0])):
        figure = plt.figure(figsize=(8.0, 5.0))
        ax = figure.add_subplot(111)

        data_column = [row[i] for row in data]
        fit = pylab.polyfit(range(len(data_column)), data_column, 4)
        fit_fn = graph_generator.gen_polinomial_function(fit)

        ax.plot(data_column, 'b.')
        p, = ax.plot(range(len(data_column)), fit_fn(range(len(data_column))), 'r-')

        if original_map and behaviors_3:
            ax.set_xlim([0,num_partidas])
        elif ( not original_map ) and behaviors_3:
            ax.axis([0, num_partidas, -10, 600])
        elif original_map and ( not behaviors_3 ):
            #ax.axis([0, num_partidas, 0, 550])
            pass
        elif ( not original_map ) and ( not behaviors_3 ):
            ax.set_xlim([0,num_partidas])
    
        ax.legend(['Amostras', u'Polinômio'])
        figure.suptitle(u'Comportamento ' + behaviors[i])
        ax.set_xlabel('Partida')
        ax.set_ylabel(u"Número de vezes escolhido")
        save_file_name = save_path + 'chosen_behaviors____pol__' + str(i) + '.eps'
        figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.linewidth'] = 0.5*matplotlib.rcParams['lines.linewidth']


def create_per_match_weights_graph(save_path, path, file_name):
    if not hasattr(create_per_match_weights_graph, "counter"):
        create_per_match_weights_graph.counter = 0  # it doesn't exist yet, so initialize it
    else:
        create_per_match_weights_graph.counter += 1

    my_file = open ( path + file_name , 'r')
    data = [ map(float, line[:-2].split(' ')) for line in my_file ]

    counter = create_per_match_weights_graph.counter

    behaviors = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Caçar']
    behaviors_ascii = ['Ficar parado', 'Comer', 'Fugir', u'Comer Capsula', u'Cacar']
    behavior = behaviors[counter]
    behavior_ascii = behaviors_ascii[counter]
    weights = [u'Bias', u'Dist. Comida', u'Dist. Capsula', u'Prob. Existir Capsula', u'Prob. Existir Fant. Branco', 
                                u'Prob. Fantasma Perto', u'Prob. Fantasma Branco Perto']

    figure = plt.figure(figsize=(8.0, 5.0))
    ax = figure.add_subplot(111)
    ax.plot(data, '-')

    matplotlib.rcParams['lines.markersize'] = 0.25*matplotlib.rcParams['lines.markersize']

    if original_map and behaviors_3:
        ax.set_xlim([0,num_partidas])
    elif ( not original_map ) and behaviors_3:
        ax.axis([0, num_partidas, -250, 150])
    elif original_map and ( not behaviors_3 ):
        #ax.axis([0, num_partidas, 0, 550])
        pass
    elif ( not original_map ) and ( not behaviors_3 ):
        ax.set_xlim([0, num_partidas])
    
    ax.legend(weights)
    figure.suptitle(u'Pesos ' + behavior)
    ax.set_xlabel('Partida')
    ax.set_ylabel(u"Valor")
    save_file_name = save_path + 'per_match_weights__' + behavior_ascii + '.eps'
    figure.savefig(save_file_name, dpi=400)

    plt.close(figure)

    matplotlib.rcParams['lines.markersize'] = 4*matplotlib.rcParams['lines.markersize']

    return data



def create_weights_graph(save_path, weights_data):

    print 'num behaviors ', len(weights_data)

    if len(weights_data) == 3:
        behaviors = ['Ficar parado', 'Comer', 'Fugir']
        weights_file = ['Bias', 'DistComida', 'ProbFantasma']
        if original_map:
            weights = ['Bias', 'Prox. Comida', 'Prob. Fantasma']
        else:
            weights = ['Bias', 'Dist. Comida', 'Prob. Fantasma']
    else:
        behaviors = ['Ficar parado', 'Comer', 'Fugir', 'Comer Capsula', u'Caçar']
        weights = ['Bias', 'Prox. Comida', 'Prox. Capsula', 'Prob. Existir Capsula', 'Prob. Existir Fant. Branco', 
                                                            'Prob. Fantasma Perto', 'Prob. Fantasma Branco Perto']
        weights_file = ['Bias', 'DistComida', 'DistCapsula', 'ProbExistirCapsula', 'ProbExistirFantBranco', 
                                                            'ProbFantasmaPerto', 'ProbFantasmaBrancoPerto']
        if original_map:
            weights = ['Bias', 'Prox. Comida', 'Prob. e Prox. Capsula', 'Prob. Existir Fant. Branco', 
                                                            'Prob. Fantasma Perto', 'Prob. Fantasma Branco Perto']
        else:
            weights = ['Bias', 'Prox. Comida', 'Prox. Capsula', 'Prob. Existir Capsula', 'Prob. Existir Fant. Branco', 
                                                            'Prob. Fantasma Perto', 'Prob. Fantasma Branco Perto']
        

    matplotlib.rcParams['lines.markersize'] = 0.25*matplotlib.rcParams['lines.markersize']

    for i in range(len(weights_data[0][0])):
        figure = plt.figure(figsize=(8.0, 5.0))
        ax = figure.add_subplot(111)

        single_weight_column = []

        for behavior_data in weights_data:
            behavior_weight_data = [row[i] for row in behavior_data[:num_partidas]]
            single_weight_column.append(behavior_weight_data)

        single_weight_column = map(list, zip(*single_weight_column))

        ax.plot(single_weight_column, '-')

        if original_map and behaviors_3:
            if i == 0:
                ax.axis([0, num_partidas, -20, 250])
            if i == 1:
                ax.axis([0, num_partidas, -10, 110])
        elif ( not original_map ) and behaviors_3:
            if i == 0 and len(behaviors) == 3:
                ax.axis([0, num_partidas, -50, 30])
            if i == 1 and len(behaviors) == 3:
                ax.axis([0, num_partidas, -15, 5])
        elif original_map and ( not behaviors_3 ):
            if i == 0:
                ax.axis([0, num_partidas, -20, 210])
            if i == 2:
                ax.axis([0, num_partidas, -5, 40])
            if i == 3:
                ax.axis([0, num_partidas, -10, 100])
            if i == 4:
                ax.axis([0, num_partidas, -90, 30])
            pass
        elif ( not original_map ) and ( not behaviors_3 ):
            if i == 0:
                ax.axis([0, num_partidas, -20, 70])
            if i == 1:
                ax.axis([0, num_partidas, -10, 80])
            if i == 2:
                ax.axis([0, num_partidas, -15, 105])
            if i == 3:
                ax.axis([0, num_partidas, -15, 50])
    

        if ( not original_map ) and ( not behaviors_3 ) and i != 3 and i != 5:
            ax.legend(behaviors, loc=2)
        else:
            if ( not original_map ) and behaviors_3 and i == 0:
                ax.legend(behaviors, loc=2)
            else:
                if original_map and ( not behaviors_3 ) and ( i == 0 or i == 1 or i == 2 or i == 3 or i == 5 ):
                    ax.legend(behaviors, loc=2)
                else:
                    ax.legend(behaviors)

        figure.suptitle(weights[i])
        ax.set_xlabel('Partida')
        ax.set_ylabel("Valor de $\omega_" + str(i+1) + "$")
        save_file_name = save_path + 'weights____pol__' + str(weights_file[i]) + '.eps'
        figure.savefig(save_file_name, dpi=400)

    matplotlib.rcParams['lines.markersize'] = 4*matplotlib.rcParams['lines.markersize']

def create_graphs_directory(path):
    directory = path + 'final_graphs/'
    if not os.path.exists(directory):
        os.makedirs(directory)
    return directory

def get_data():
    path = graph_generator.get_path_to_package('log_42_5_behaviors_6_features_original_10/')
    save_path = create_graphs_directory(path)
    log_files = graph_generator.get_log_files(path)
    log_files.sort()

    weight_data = []

    for log_file in log_files:
        if log_file.endswith('.txt'):
            if log_file.startswith('match_behaviors_'):
                create_chosen_behaviors_graph(save_path, path, log_file)
                create_chosen_behaviors_pol_graph(save_path, path, log_file)
                create_chosen_behaviors_pol_graph2(save_path, path, log_file)
            if log_file.startswith('match_scores_'):
                get_score_mean(path, log_file)
                create_score_graph(save_path, path, log_file)
            if log_file.startswith('per_match_behavior'):
                behavior_data = create_per_match_weights_graph(save_path, path, log_file)
                weight_data.append(behavior_data)

    create_weights_graph(save_path, weight_data)


if __name__ == '__main__':
    try:
        get_data()
    except rospy.ROSInterruptException: pass