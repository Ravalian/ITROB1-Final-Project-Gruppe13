try:
    from python_qt_binding.QtCore import qWarning
    from python_qt_binding.QtGui import QWidget, QFrame,QVBoxLayout 
except:
    from PyQt5.QtCore import qWarning
    from PyQt5 import  QtWidgets #works for pyqt5

class Gui():
    
    def __init__(self,name,layout,components):
        self.name = name
        self.layout = layout
        self.components = []
        for c in components:
            self.components.append(c)
            
    def traverse(self,traverser):
        traverser.beginGui(self)
        for c in self.components:
            c.traverse(traverser)
        traverser.endGui(self)
    
    def to_str(self):
        s = self.name + "\n"
        level = 1
        for c in self.components:
            s += c.to_str(level) + "\n"
            
        return s

        
