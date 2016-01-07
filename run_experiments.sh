#!/bin/sh
PORT=5555

OUTPUT_PATH=experiments
LEARNING_GAMES=100
TESTING_GAMES=15
NUM_REPETITIONS=10

OUTPUT_1=$OUTPUT_PATH/noiseless/ghost_ai
OUTPUT_2=$OUTPUT_PATH/noise/ghost_ai
OUTPUT_3=$OUTPUT_PATH/noiseless/ghost_ai_pacman_eater
OUTPUT_4=$OUTPUT_PATH/noise/ghost_ai_pacman_eater

# while [[ $# > 1 ]]
# do
# key="$1"

# case $key in
#   --port)
#   PORT="$2"
#   shift
#   ;;
# esac
# shift
# done

echo "Connecting via TCP port ${PORT}"

# rm -rf $OUTPUT_PATH
# mkdir $OUTPUT_PATH
# mkdir -p $OUTPUT_1
# mkdir -p $OUTPUT_2
# mkdir -p $OUTPUT_3
# mkdir -p $OUTPUT_4

# AI Ghost vs. Random Pacman - Noiseless
# for i in `seq 8 $NUM_REPETITIONS`
# do
#   python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 2 -o $OUTPUT_1/results2_$i.txt -p $OUTPUT_1/policies2_$i.txt
  # python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 3 -o $OUTPUT_1/results3_$i.txt -p $OUTPUT_1/policies3_$i.txt
  # python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 4 -o $OUTPUT_1/results4_$i.txt -p $OUTPUT_1/policies4_$i.txt
# done

# AI Ghost vs. Random Pacman - Noise
# for i in `seq 2 $NUM_REPETITIONS`
# do
#   python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 2 -o $OUTPUT_2/results2_$i.txt -p $OUTPUT_2/policies2_$i.txt --noise 3
  # python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 3 -o $OUTPUT_2/results3_$i.txt -p $OUTPUT_2/policies3_$i.txt --noise 3
  # python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent random --ghost-agent ai -e 4 -o $OUTPUT_2/results4_$i.txt -p $OUTPUT_2/policies4_$i.txt --noise 3
# done

# AI Ghost vs. Greedy Pacman - Noiseless
# for i in `seq $NUM_REPETITIONS`
# do
#   python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent eater --ghost-agent ai -e 2 -o $OUTPUT_3/results2_$i.txt -p $OUTPUT_3/policies2_$i.txt -g
#   python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent eater --ghost-agent ai -e 3 -o $OUTPUT_3/results3_$i.txt -p $OUTPUT_3/policies3_$i.txt
#   python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent eater --ghost-agent ai -e 4 -o $OUTPUT_3/results4_$i.txt -p $OUTPUT_3/policies4_$i.txt
# done

# AI Ghost vs. Greedy Pacman - Noise
for i in `seq 6 $NUM_REPETITIONS`
do
  python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent eater --ghost-agent ai -e 2 -o $OUTPUT_4/results2_$i.txt -p $OUTPUT_4/policies2_$i.txt --noise 3
#   python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent eater --ghost-agent ai -e 3 -o $OUTPUT_4/results3_$i.txt -p $OUTPUT_4/policies3_$i.txt --noise 3
#   python simulator.py --port $PORT -l $LEARNING_GAMES -t $TESTING_GAMES --pacman-agent eater --ghost-agent ai -e 4 -o $OUTPUT_4/results4_$i.txt -p $OUTPUT_4/policies4_$i.txt --noise 3
done
