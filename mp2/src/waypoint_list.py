import numpy as np    
import numpy as np
import matplotlib.pyplot as plt



class WayPoints():
    def __init__(self):
        
        # Set 1 (Bezier)
        # self.startPoint = (0,0)
        # self.endPoint = (100, 100)
        # self.controlPoints = [(17,68), (109, 53)]
        # t_values = np.linspace(0, 1, 100)
        # self.pos_list = [self.cubic_bezier(self.startPoint, self.controlPoints[0], self.controlPoints[1], self.endPoint, t) for t in t_values]

        # Set 2 (Bezier)
        # self.startPoint = (0,0)
        # self.endPoint = (100, 100)
        # self.controlPoints = [(25,70), (44, 34)]
        # t_values = np.linspace(0, 1, 30)
        # self.pos_list = [self.cubic_bezier(self.startPoint, self.controlPoints[0], self.controlPoints[1], self.endPoint, t) for t in t_values]
        
        
        # Set 3 (Arc)
        self.startPoint = np.array([0,0])
        self.endPoint = np.array([30,0])
        self.pos_list = self.generate_steep_arc(self.startPoint, self.endPoint)


    def cubic_bezier(self, p0, p1, p2, p3, t):
        """ Calculate the cubic Bezier curve point.
        
        Args:
        p0 (tuple): Starting point (x, y).
        p1 (tuple): First control point (x, y).
        p2 (tuple): Second control point (x, y).
        p3 (tuple): Ending point (x, y).
        t (float): Parameter t, ranges from 0 to 1.
        
        Returns:
        tuple: Point on the Bezier curve for parameter t.
        """
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        y = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        return (x, y)


    def plotBezierPoints(self):
        
        # Unpack points for plotting
        x_values, y_values = zip(*self.pos_list)

        # Plotting the curve
        plt.figure(figsize=(8, 4))
        
        # Bezier Curve
        plt.scatter(x_values, y_values, label="Cubic Bezier Curve")
        plt.scatter([self.startPoint[0], self.controlPoints[0][0], self.controlPoints[1][0], self.endPoint[0]], 
                    [self.startPoint[1], self.controlPoints[0][1], self.controlPoints[1][1], self.endPoint[1]], 
                    color='red', label="Control Points")
        
 
        plt.legend()
        plt.grid(True)
        plt.title("Cubic Bezier Curve with Two Control Points")
        plt.show()


    def getWayPoints(self):
        return self.pos_list