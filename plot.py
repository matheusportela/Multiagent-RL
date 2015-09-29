import matplotlib.pylab as plt

if __name__ == '__main__':
    learn_scores = []
    test_scores = []

    with open('learn_scores.txt') as f:
        for line in f:
            learn_scores.append(float(line))

    with open('test_scores.txt') as f:
        for line in f:
            test_scores.append(float(line))

    plt.scatter(range(len(learn_scores)), learn_scores, c='b')
    plt.scatter(range(len(learn_scores), len(learn_scores) + len(test_scores)), test_scores, c='g')
    plt.show()