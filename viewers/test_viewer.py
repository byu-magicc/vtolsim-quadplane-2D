
import pyqtgraph.opengl as gl
import numpy as np
import pyqtgraph as pg

class TestViewer:

    def __init__(self):

        self.app = pg.QtWidgets.QApplication([]) 

        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Not Quadplane Viewer')
        self.window.setGeometry(0, 0, 500, 500)  # args: upper_left_x, upper_right_y, width, height

        self.scale = 2500

        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(20, 20, 20) # set the size of the grid (distance between each line)
        self.window.addItem(grid)

        center = self.window.cameraPosition()
        center.setX(1000)
        center.setY(1000)
        center.setZ(0)
        self.window.setCameraPosition(pos=center,
                                      distance=self.scale,
                                      elevation=50,
                                      azimuth=-90)
        self.window.setBackgroundColor('k')

        mesh = [np.array([[0.0],[0.0],[0.0]]),
                np.array([[200.0],[0.0],[0.0]]),
                np.array([[0.0],[200.0],[0.0]])]
        
        meshColor = np.array([0., 1., 0., 1])


        self.groundMesh = gl.GLMeshItem(
            vertexes = mesh,
            vertexColors=meshColor,
            drawEdges=True,
            smooth=False,
            computeNormals=False
        )
        self.groundMesh.setGLOptions('translucent')
        self.window.addItem(self.groundMesh)
        self.window.show()
        self.window.raise_()