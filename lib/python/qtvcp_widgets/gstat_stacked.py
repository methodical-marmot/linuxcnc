#!/usr/bin/python2.7

from PyQt4 import QtCore, QtGui

from qtvcp_widgets.simple_widgets import _HalWidgetBase
from qtvcp.qt_glib import GStat
GSTAT = GStat()

class GstatStacked(QtGui.QStackedWidget, _HalWidgetBase):
    def __init__(self, parent=None):
        super(GstatStacked, self).__init__(parent)
        self.auto = True
        self.mdi = True
        self.manual = True

    def _hal_init(self):
        def _switch( index):
            print 'index',index
            self.setCurrentIndex(index)
        if self.auto:
            GSTAT.connect('mode-auto', lambda w: _switch(2))
            self.addWidget()
        if self.mdi:
            GSTAT.connect('mode-mdi', lambda w: _switch(1))
            self.addWidget()
        if self.manual:
            GSTAT.connect('mode-manual', lambda w: _switch(0))
            self.addWidget()
