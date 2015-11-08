import csv
import pickle
import matplotlib.pylab as plt
import numpy as np

def load_results(filename):
    with open(filename) as f:
        results = pickle.loads(f.read())
    return results

def calculate_regression_coefficients(data, degree=5):
    return np.polyfit(range(len(data)), data, degree)

def calculate_regression_y(x, coeff):
    return np.sum([x**(len(coeff) - i - 1) * coeff[i] for i in range(len(coeff))])

def load_behavior_count(filename):
    behavior_count = []
    behavior_dict = {}

    with open(filename) as f:
        reader = csv.DictReader(f)
        for row in reader:
            behavior_count.append(row)

    for behavior in behavior_count[0]:
        behavior_dict[behavior] = []

    # Calculating the behavior probability of being selected
    for behavior in behavior_dict:
        for data in behavior_count:
            behavior_sum = sum([int(v) for v in data.values()])
            behavior_prob = float(data[behavior]) / behavior_sum
            behavior_dict[behavior].append(behavior_prob)

    return behavior_dict

def plot_scores(learn_scores, test_scores):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    data = learn_scores + test_scores
    coeff = calculate_regression_coefficients(data, degree=5)
    regression = [calculate_regression_y(x, coeff) for x in range(len(data))]

    print 'Regression coefficients:', coeff

    ax.scatter(range(len(learn_scores)), learn_scores, c='b')
    ax.scatter(range(len(learn_scores), len(learn_scores) + len(test_scores)), test_scores, c='g')
    ax.plot(regression, c='r')

def plot_behavior_count(behavior_count):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.ylabel('Probability of selecting the behavior')
    plt.xlabel('Number of games')
    plt.ylim([0, 1])
    colors = ['r', 'g', 'b']

    for i, behavior in enumerate(behavior_count):
        coeff = calculate_regression_coefficients(behavior_count[behavior], degree=5)
        regression = [calculate_regression_y(x, coeff) for x in range(len(behavior_count[behavior]))]
        ax.plot(regression, label=behavior, c=colors[i], linewidth=2.0)
        ax.scatter(range(len(behavior_count[behavior])), behavior_count[behavior], c=colors[i], alpha=0.10)

    ax.legend()

if __name__ == '__main__':
    results = load_results('results.txt')
    behavior_count = load_behavior_count('results/behavior_count.txt')

    plot_scores(learn_scores, test_scores)
    plot_behavior_count(behavior_count)
    plt.show()