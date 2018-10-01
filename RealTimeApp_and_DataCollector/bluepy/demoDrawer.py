import matplotlib.pyplot as plt
import warnings

from scipy.misc import imread


class demoDrawer():
    #Suppose we know the x range:
 
    def __init__(self,xdata,ydata):
        self.xdata,self.ydata = xdata,ydata
        #Set up plot
        self.figure, self.ax = plt.subplots(figsize = (15,12))

        self.lines, = self.ax.plot([],[], 'o')

        mngr = plt.get_current_fig_manager()
        mngr.window.wm_geometry(("+0+0"))

        img = imread("LGroundTruth02.png")
        
        layoutrange = [-2950,2850,-1050,4950]
        self.ax.axis(layoutrange)
        self.ax.imshow(img,extent = layoutrange)

        warnings.filterwarnings("ignore",".*GUI is implemented.*")
        
        self.on_running()

    def on_running(self):
        #Update data (with the new _and_ the old points)
        while True:

            self.lines.set_xdata(self.xdata)
            self.lines.set_ydata(self.ydata)

            plt.pause(0.1)

