import os, sys
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[1]))
from planners.takeoffGenerator import flightPathGenerator, pathTypes



testPoint = 0
