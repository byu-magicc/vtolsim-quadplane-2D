#setup your environment for python

pip3 install --upgrade pip
pip3 install numpy
pip3 install scipy
pip3 install matplotlib
pip3 install pyqtgraph
pip3 install pyqt6
pip3 install pyopengl
pip3 install pynput

#sometimes there are issues with that data_viewer if you have ROS installed on your system, in this case. If you are having problems
#with the data viewer try running the following in the commandline and then reinstalling pqt6. 
sudo apt autoremove pyqt6*

#If you still have problems try cleaning your python environment or using a virtual environment.

# To run the video writer install the following
pip3 install opencv-python
pip3 install Pillow


####### Creating a python virtual environment. #######

# this is how I created my virtual environment (used for many simulation)
python -m venv /User/beard/repositories/.venv

# activite virtual environment using
source /User/beard/repositories/.venv/bin/activite

# close environemnt
deactivate


