import matplotlib.pylab as plt

if __name__ == '__main__':
    scores = []

    with open('scores.txt') as f:
        for line in f:
            scores.append(float(line))

    plt.scatter(range(len(scores)), scores)
    plt.show()