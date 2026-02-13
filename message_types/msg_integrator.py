import numpy as np


class MsgIntegrator:

    def __init__(self,
                 dimension: int = 2):

        self.pos_error_int = np.zeros((dimension, 1))

    def setIntegrator(self,
                      integratorValue: np.ndarray):
        self.pos_error_int = integratorValue

    def getIntegrator(self):
        return self.pos_error_int