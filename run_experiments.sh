#!/bin/sh
OUTPUT_PATH=experiments
LEARNING_GAMES=15
TESTING_GAMES=15

rm $OUTPUT_PATH/pacman_ai/*
rm $OUTPUT_PATH/ghost_ai/*
rm $OUTPUT_PATH/both_ai/*

python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost random -e 1 -o $OUTPUT_PATH/pacman_ai/results1.txt -p $OUTPUT_PATH/pacman_ai/policies1.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman random --ghost ai -e 1 -o $OUTPUT_PATH/ghost_ai/results1.txt -p $OUTPUT_PATH/ghost_ai/policies1.txt

python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost random -e 2 -o $OUTPUT_PATH/pacman_ai/results2.txt -p $OUTPUT_PATH/pacman_ai/policies2.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman random --ghost ai -e 2 -o $OUTPUT_PATH/ghost_ai/results2.txt -p $OUTPUT_PATH/ghost_ai/policies2.txt

python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost random -e 3 -o $OUTPUT_PATH/pacman_ai/results3.txt -p $OUTPUT_PATH/pacman_ai/policies3.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman random --ghost ai -e 3 -o $OUTPUT_PATH/ghost_ai/results3.txt -p $OUTPUT_PATH/ghost_ai/policies3.txt

python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost random -e 4 -o $OUTPUT_PATH/pacman_ai/results4.txt -p $OUTPUT_PATH/pacman_ai/policies4.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman random --ghost ai -e 4 -o $OUTPUT_PATH/ghost_ai/results4.txt -p $OUTPUT_PATH/ghost_ai/policies4.txt

python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost ai -e 1 -o $OUTPUT_PATH/both_ai/results1.txt -p $OUTPUT_PATH/both_ai/policies1.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost ai -e 2 -o $OUTPUT_PATH/both_ai/results2.txt -p $OUTPUT_PATH/both_ai/policies2.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost ai -e 3 -o $OUTPUT_PATH/both_ai/results3.txt -p $OUTPUT_PATH/both_ai/policies3.txt
python simulator.py -l $LEARNING_GAMES -t $TESTING_GAMES --pacman ai --ghost ai -e 4 -o $OUTPUT_PATH/both_ai/results4.txt -p $OUTPUT_PATH/both_ai/policies4.txt