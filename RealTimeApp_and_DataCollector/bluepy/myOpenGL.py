from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import os
import threading
 
ESCAPE = '\033'

cubeList = [] # for demo in this code
count = 1

class myDraw:
 
    def __init__(self,nodeList):

        self.nodeList = nodeList
        self.removeList = [] #delete the node disconnected
        self.window = []   
        self.serialNum = 2

        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(0,0)
        glutInitWindowPosition(0,0)

    def addFirstDevice(self):
        windowName = 'OpenGL Python Cube 1'
        node = self.nodeList[0]
        glutInitWindowPosition(370*(len(self.window)),0)
        windowNum = glutCreateWindow(windowName)
        node.windowNumber = windowNum

        glutSetWindow(windowNum)
        glutDisplayFunc(node.nodeCube.DrawGLScene)
        glutKeyboardFunc(self.keyPressed)
        self.InitGL(600, 480)
        self.window.append(windowNum)

        glutIdleFunc(self.refresh)
        threading.Thread(target = glutMainLoop).start()

    def removeDevice(self, node):
        self.removeList.append(node.windowNumber)

    def InitGL(self,Width, Height): 
 
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def keyPressed(self,*args):
        if args[0] == ESCAPE:
                sys.exit()

    def delWindowNum(self,windowNum):
        self.deleteList.append(windowNum)

    def refresh(self):

        # delete window using list
        if self.removeList:
            for windowNumber in self.removeList:
                self.window.remove(windowNumber)
                glutDestroyWindow(windowNumber)
                self.removeList.remove(windowNumber)
        #     for deleteWindowNum in self.deleteList:
        #         self.window.remove(deleteWindowNum)

        #         try:
        #             print "try to destory window"
        #             glutDestroyWindow(deleteWindowNum)
        #         except:
        #             print "error"

        #         self.deleteList.remove(deleteWindowNum)

        ## update the current condition
        for wi in self.window:
            glutSetWindow(wi)
            glutPostRedisplay()

        # find new node, make a new window
        if(len(self.window) < len(self.nodeList)):
            if len(self.nodeList) != 2:
                return
            for node in self.nodeList:
                if node.windowNumber == None:
                    windowName = 'OpenGL Python Cube %d' % self.serialNum
                    glutInitWindowPosition(370*(len(self.window)),0)
                    windowNum = glutCreateWindow(windowName)
                    node.windowNumber = windowNum
                    glutSetWindow(windowNum)
                    glutDisplayFunc(node.nodeCube.DrawGLScene)
                    glutKeyboardFunc(self.keyPressed)
                    self.InitGL(600, 480)
                    self.window.append(windowNum)
                    self.serialNum = self.serialNum + 1

class myCube:

    def __init__(self,num = 0,x_axis = 0.0,y_axis = 0.0,z_axis = 0.0):

        self.num = num     
        self.angle =[0.0,0.0,0.0]

        # self.angle[0] =  z_axis
        # self.angle[1]  = y_axis
        # self.angle[2]  = x_axis

        self.angle[0] =  z_axis
        self.angle[1]  = y_axis
        self.angle[2]  = x_axis

    def DrawGLScene(self):
           
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
         
        glLoadIdentity()
        glTranslatef(0.0,0.0,-6.0)
        glRotatef(self.angle[0] ,0.0,-1.0,0.0)
        glRotatef(self.angle[1] ,-1.0,0.0,0.0)
        glRotatef(self.angle[2] ,0.0,0.0,-1.0)        
        
        
        #glRotatef(90 ,0.0,1.0,0.0)
         
            # Draw Cube (multiple quads)
        glBegin(GL_QUADS)
         
        glColor3f(0.0,1.0,0.0)#green
        glVertex3f( 2.0, 0.5,-1.0)
        glVertex3f(-2.0, 0.5,-1.0)
        glVertex3f(-2.0, 0.5, 1.0)
        glVertex3f( 2.0, 0.5, 1.0) 
         
        glColor3f(1.0,0.0,0.0)#red
        glVertex3f( 2.0,-0.5, 1.0)
        glVertex3f(-2.0,-0.5, 1.0)
        glVertex3f(-2.0,-0.5,-1.0)
        glVertex3f( 2.0,-0.5,-1.0) 
         
        glColor3f(1.0,1.0,1.0)#white
        glVertex3f( 2.0, 0.5, 1.0)
        glVertex3f(-2.0, 0.5, 1.0)
        glVertex3f(-2.0,-0.5, 1.0)
        glVertex3f( 2.0,-0.5, 1.0)
     
        glColor3f(1.0,1.0,0.0)#yellow
        glVertex3f( 2.0,-0.5,-1.0)
        glVertex3f(-2.0,-0.5,-1.0)
        glVertex3f(-2.0, 0.5,-1.0)
        glVertex3f( 2.0, 0.5,-1.0)
     
        glColor3f(0.0,0.0,1.0)#blue
        glVertex3f(-2.0, 0.5, 1.0) 
        glVertex3f(-2.0, 0.5,-1.0)
        glVertex3f(-2.0,-0.5,-1.0) 
        glVertex3f(-2.0,-0.5, 1.0) 
     
        glColor3f(1.0,0.0,1.0)#purple
        glVertex3f( 2.0, 0.5,-1.0) 
        glVertex3f( 2.0, 0.5, 1.0)
        glVertex3f( 2.0,-0.5, 1.0)
        glVertex3f( 2.0,-0.5,-1.0)
        glEnd() 
        glutSwapBuffers()


def Demomain():
        cube0 = myCube(0)
        cube1 = myCube(1)

        cubeList.append(cube0)
        cubeList.append(cube1)

        if len(cubeList) > 0:
            mydraw = myDraw()

        while(len(cubeList) < 10):
            cubeList.append(myCube(len(cubeList)+1))
        print("exit while loop ,len = %d" % len(cubeList))


if __name__ == "__main__":
        Demomain() 
