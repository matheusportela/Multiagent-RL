#!/bin/sh
OUTPUT_PATH=experiments
LEARNING_GAMES=100
TESTING_GAMES=15

rm -rf $OUTPUT_PATH
mkdir $OUTPUT_PATH
mkdir $OUTPUT_PATH/ghost_ai
mkdir $OUTPUT_PATH/ghost_ai_noise

python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 2 -o $OUTPUT_PATH/ghost_ai/results2.txt -p $OUTPUT_PATH/ghost_ai/policies2.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 3 -o $OUTPUT_PATH/ghost_ai/results3.txt -p $OUTPUT_PATH/ghost_ai/policies3.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 4 -o $OUTPUT_PATH/ghost_ai/results4.txt -p $OUTPUT_PATH/ghost_ai/policies4.txt

python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 2 -o $OUTPUT_PATH/ghost_ai_noise/results2.txt -p $OUTPUT_PATH/ghost_ai_noise/policies2.txt --noise 3
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 3 -o $OUTPUT_PATH/ghost_ai_noise/results3.txt -p $OUTPUT_PATH/ghost_ai_noise/policies3.txt --noise 3
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 4 -o $OUTPUT_PATH/ghost_ai_noise/results4.txt -p $OUTPUT_PATH/ghost_ai_noise/policies4.txt --noise 3