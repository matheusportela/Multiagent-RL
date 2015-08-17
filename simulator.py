from simulator import pacman as pacman_simulator
from simulator import layout
from simulator import textDisplay
from simulator import graphicsDisplay
from simulator import myAgents

if __name__ == '__main__':
    layout_file = 'mediumClassic'
    layout = layout.getLayout(layout_file)
    if layout == None: raise Exception("The layout " + layout_file + " cannot be found")
    pacman = myAgents.RandomPacmanAgent()
    num_ghosts = 4
    ghosts = [myAgents.RandomGhostAgent(i+1) for i in range(num_ghosts)]
    is_text_only = False
    if is_text_only:
        display = textDisplay.PacmanGraphics()
    else:
        zoom = 1.0
        display = graphicsDisplay.PacmanGraphics(zoom, frameTime=0.1)
    numGames = 1
    record = False
    pacman_simulator.runGames(layout, pacman, ghosts, display, numGames, record)