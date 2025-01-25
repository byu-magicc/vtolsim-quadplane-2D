import numpy as np

#This file implements the part of the contoller, which 
#allocates forces and torques desired for the control of the aircraft
#This controller is significantly simpler than the previously implemented allocation
#Technique, thankfully

#creates the Low Level controller. Unlike past controllers, this particular
#controller will NOT take into account gravity. That is how Dr. Beard likes it,
#so we will not.
class lowLevelControl:

    #creates the initialization function