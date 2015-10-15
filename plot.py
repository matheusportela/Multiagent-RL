import csv
import matplotlib.pylab as plt
import numpy as np

def load_scores(filename):
    scores = []

    with open(filename) as f:
        for line in f:
            scores.append(float(line))

    return scores

def calculate_regression_coefficients(data, degree=5):
    return np.polyfit(range(len(data)), data, degree)

def calculate_regression_y(x, coeff):
    return np.sum([x**(len(coeff) - i - 1) * coeff[i] for i in range(len(coeff))])

def load_behavior_count(filename):
    behavior_count = []

    with open(filename) as f:
        reader = csv.DictReader(f)
        for row in reader:
            behavior_count.append(row)

    return behavior_count

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

    for behavior in behavior_count[0]:
        ax.plot([b[behavior] for b in behavior_count], label=behavior)

    ax.legend()

if __name__ == '__main__':
    learn_scores = load_scores('learn_scores.txt')
    test_scores = load_scores('test_scores.txt')
    behavior_count = load_behavior_count('behavior_count.txt')

    plot_scores(learn_scores, test_scores)
    plot_behavior_count(behavior_count)
    plt.show()