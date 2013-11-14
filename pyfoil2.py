#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      NaotoMORITA
#
# Created:     08/11/2013
# Copyright:   (c) NaotoMORITA 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------

#-*- coding: utf-8 -*-
#------鬩怜遜・ｽ・ｼ髯懷姓・ｹ譎｢・ｽ蟶晏擅繝ｻ・ｭ驍ｵ・ｺ繝ｻ・ｿ鬮ｴ雜｣・ｽ・ｼ驛｢・ｧ髦ｮ蜷ｶﾂ遏･atplotlib驍ｵ・ｺ繝ｻ・ｧ鬮ｯ・ｦ繝ｻ・ｨ鬩穂ｼ夲ｽｽ・ｺ驍ｵ・ｺ陷ｷ・ｶ繝ｻ繝ｻ
import numpy
import scipy


import sys, os, random
from PyQt4 import QtGui, QtCore

# Matplotlib Figure object
import matplotlib
import matplotlib.pyplot

# Python Qt4 bindings for GUI objects
import PyQt4.QtGui

# import the Qt4Agg FigureCanvas object, that binds Figure to
# Qt4Agg backend. It also inherits from QWidget
#from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.backends.backend_qt4agg
import matplotlib.backends.backend_agg

progname = os.path.basename(sys.argv[0])
progversion = "0.1"

class Matplot(matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg):
    def __init__(self, parent=None, width=6, height=3, dpi=50, Fx = numpy.array([[0],[0]]), Fy = numpy.array([[0],[0]])):
        self.Fx = numpy.array(Fx)
        self.Fy = numpy.array(Fy)

        fig = matplotlib.figure.Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        self.axes.hold(False)
        self.axes.set_aspect("equal")
        self.axes.set_xlim(0,1)
        self.compute_initial_figure()

        matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg.__init__(self, fig)
        self.setParent(parent)

        matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg.setSizePolicy(self,
                                   QtGui.QSizePolicy.Expanding,
                                   QtGui.QSizePolicy.Expanding)
        matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg.updateGeometry(self)

    def compute_initial_figure(self):
        pass


class FoilPlot(Matplot):
    """A canvas that updates itself every second with a new plot."""
    def __init__(self, *args, **kwargs):
        Matplot.__init__(self, *args, **kwargs)

    def compute_initial_figure(self):
        self.load()
        self.axes.plot(self.Fx, self.Fy)

    def update_figure(self):

        self.load()
        self.axes.plot(self.Fx, self.Fy)
        self.draw()


    def load(self):
        filename = QtGui.QFileDialog.getOpenFileName()
        foil = numpy.loadtxt(filename)
        self.Fx = foil[:,0]
        self.Fy = foil[:,1]

class MatPlotWidget(QtGui.QWidget):
    def __init__(self,parent = None):
        QtGui.QWidget.__init__(self, parent = parent)

        self.mpl = FoilPlot()
        layout = QtGui.QVBoxLayout(self)
        layout.addWidget(self.mpl)
        self.setLayout(layout)


class ButtonWidget(QtGui.QWidget):
    def __init__(self,parent = None, Fx = numpy.array([[0],[0]]), Fy = numpy.array([[0],[0]])):
        QtGui.QWidget.__init__(self, parent = parent)
        self.openbutton = QtGui.QPushButton("OPEN Foil",parent = self)

        layout = QtGui.QGridLayout()
        layout.addWidget(self.openbutton)
        self.setLayout(layout)


def main():
    qApp = QtGui.QApplication(sys.argv)

    panel = QtGui.QWidget()

    mpw = MatPlotWidget(parent = panel)
    button_panel = ButtonWidget(parent = panel)

    panel_layout = QtGui.QVBoxLayout()
    panel_layout.addWidget(mpw)
    panel_layout.addWidget(button_panel)
    panel.setLayout(panel_layout)
    panel.setFixedSize(680,200)



    mainwindow = QtGui.QMainWindow()
    mainwindow.setCentralWidget(panel)
    mainwindow.show()

    button_panel.connect(button_panel.openbutton,QtCore.SIGNAL('clicked()'),mpw.mpl.update_figure)


    sys.exit(qApp.exec_())



if __name__ == '__main__':
    main()
