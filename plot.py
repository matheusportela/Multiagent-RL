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

if __name__ == '__main__':
    learn_scores = load_scores('learn_scores.txt')
    test_scores = load_scores('test_scores.txt')
    data = learn_scores + test_scores
    coeff = calculate_regression_coefficients(data, degree=5)
    regression = [calculate_regression_y(x, coeff) for x in range(len(data))]

    print 'Regression coefficients:', coeff

    plt.scatter(range(len(learn_scores)), learn_scores, c='b')
    plt.scatter(range(len(learn_scores), len(learn_scores) + len(test_scores)), test_scores, c='g')
    plt.plot(regression, c='r')
    plt.show()