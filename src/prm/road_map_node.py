class PRM_Node:

    def __init__(self, x = 0, y = 0, parent = None, children = [], index = 0):
        self.x = x
        self.y = y
        self.parent = parent
        self.children = children
        self.index = index #frankly, pretty useless if we use strict linked graph
        #Feel free to add other member variables if you think you need!

    def addChild(self, childNode):
        self.children.append(childNode)
        #python doesn't have the concept of pointer, it shows drawback here!!!
        #Yes, we like C!
        childNode.parent = self
