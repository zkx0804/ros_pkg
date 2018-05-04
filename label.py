import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.colors as colors


grid_size = 100

class Test():
    
    
    def xy2mapIndex(self, x,y):
        """ Converts the x,y grid coordinates into a row-major index for the 
            map. """
        if x>self.ogrid_sizeX or y>self.ogrid_sizeY:
            print('MAP IS TOO SMALL!!!')
            return self.ogrid_sizeX * self.ogrid_sizeY -1
        else:
            return int(y*self.ogrid_sizeY + x)
        
        
    def mapIndex2xy(self, index):
        """ Converts the row-major index for the map into x,y coordinates."""
        x = np.mod(index, self.ogrid_sizeY)
        y = (index-x)/self.ogrid_sizeY
        return x,y    
    
    def createFakeData(self):
        # grid x,y
        x,y = 0,0
        
        # Crate unknown space
        size = 30
        points = []
        x_list = []
        y_list = []
        for i in range(size):
            y = i
            for j in range (i):
                x = j
                x_list.append(x)
                y_list.append(y)
                index = self.xy2mapIndex(x,y)
                points.append([x,y])
                self.fake_map_data[index] = -1
        
        print(points)
        print(self.fake_map_data)        
    
    
    def labelFunc(self):
        # grid x,y
        x,y = 0,0
        
        # Crate unknown space
        size = 20
        points = []
        obs = []
        for i in range(size):
            y = i
            for j in range (i):
                x = j
                if x == y-1 and x in range(4,7):
                    obs.append([x,y])
                    index = self.xy2mapIndex(x,y)
                    self.fake_map_data[index] = 100 
                else:
                    index = self.xy2mapIndex(x,y)
                    points.append([x,y])
                    self.fake_map_data[index] = -1                    
                
        for i in range(60):
            for j in range(21):
                index = self.xy2mapIndex(i + 30,j)
                self.fake_map_data[index] = -1
        
        for i in range(20):
            for j in range(40):
                index = self.xy2mapIndex(i+51, j + 21)
                self.fake_map_data[index] = -1
                
        for i in range(21):
            for j in range(40):
                index = self.xy2mapIndex(i+30,j+21)
                self.fake_map_data[index] = 100
        """
        for i in range(20):
            for j in range(40):
                index = self.xy2mapIndex(i+71,j+21)
                self.fake_map_data[index] = 100              
        """            
        print(points)
        #print(self.fake_map_data)
       
        
        
        #Find frontier
             
        #testlist = np.ones_like(frontier, dtype=np.int16)*-1
        #print(testlist)        
        
        #labels,equiv = self.blogDetection(frontier)
        #print(labels)
        #print(equiv)
        
        frontier = self.getFrontier()
        f1 = frontier[0]
        f1_xs = []
        f1_ys = []        
        f2 = frontier[1]
        f2_xs = []
        f2_ys = []        
        f3 = frontier[2]
        f3_xs = []
        f3_ys = []
        f4 = frontier[3]
        f4_xs = []
        f4_ys = []        
        
        
        unknown_xs = []
        unknown_ys = []
        o_xs = []
        o_ys = []
        free_xs = []
        free_ys = []
        
        # map
        for i in range(len(self.fake_map_data)):
            if self.fake_map_data[i] == -1:
                x,y = self.mapIndex2xy(i)
                unknown_xs.append(x)
                unknown_ys.append(y)
            elif self.fake_map_data[i] == 100:
                x,y = self.mapIndex2xy(i)
                o_xs.append(x)
                o_ys.append(y)
            else:
                x,y = self.mapIndex2xy(i)                
                free_xs.append(x)
                free_ys.append(y)            
        
        for j in range(len(f1)):
            x,y = self.mapIndex2xy(f1[j])
            f1_xs.append(x)
            f1_ys.append(y)
        
        for j in range(len(f2)):
            x,y = self.mapIndex2xy(f2[j])
            f2_xs.append(x)
            f2_ys.append(y)        
        
        for j in range(len(f3)):
            x,y = self.mapIndex2xy(f3[j])
            f3_xs.append(x)
            f3_ys.append(y)
            
        for j in range(len(f4)):
            x,y = self.mapIndex2xy(f4[j])
            f4_xs.append(x)
            f4_ys.append(y)        
        
        matplotlib.rcParams['axes.unicode_minus'] = False
        fig, ax = plt.subplots()
        ax.plot(unknown_xs, unknown_ys, "co", o_xs, o_ys, "ko", free_xs,free_ys, "c,", f1_xs, f1_ys, "ro", f2_xs, f2_ys, "yo", f3_xs, f3_ys, "go", f4_xs, f4_ys, "bo")
        ax.set_title('Testing')
        plt.show()           
        
        print ("done!")
        
        
        
    def onFrontier(self, index):
        """ This function takes in an index of a cell in the map and determines
            if it is part of the frontier. The frontier is defined as regions
            on the border of empty space and unknown space.
        """
        connected = False
        top = False
        bottom = False

        # Skip the known cell
        if self.fake_map_data[index] < 0 or self.fake_map_data[index] >= 90:
            return connected
        
        x,y = self.mapIndex2xy(index)
        
        # Check the cell above to see if its connected to a known 
        #   cell
        # x, y + 1
        if y + 1 < self.ogrid_sizeY:
            ac = self.xy2mapIndex(x,y+1)
            if self.fake_map_data[ac] == -1:
                return True
            elif self.fake_map_data[ac] > 90:
                return connected            


        # Check the cell below to see if its connected to a known 
        #   cell
        # x, y -1
        if y -1 >= 0:
            bc = self.xy2mapIndex(x,y-1)
            if self.fake_map_data[bc] == -1:
                return True
            elif self.fake_map_data[bc] > 90:
                return connected            
        


        # Check the cell to the left to see if its connected to a  
        #   known cell
        # x - 1, y
        if x -1 >= 0:
            lc = self.xy2mapIndex(x -1 ,y)
            if self.fake_map_data[lc] == -1:
                return True
            elif self.fake_map_data[lc] > 90:
                return connected            
            
        # Check top left
        # x - 1, y + 1
        if x - 1 >= 0 and y + 1 < self.ogrid_sizeY:
            tl = self.xy2mapIndex(x - 1, y + 1)
            if self.fake_map_data[tl] == -1:
                return True
            elif self.fake_map_data[tl] > 90:
                return connected            
        

        # Check bottom left
        # x - 1, y - 1
        if x - 1 >= 0 and y - 1 >= 0:
            bl = self.xy2mapIndex(x - 1, y - 1)
            if self.fake_map_data[bl] == -1:
                return True
            elif self.fake_map_data[bl] > 90:
                return connected            

        # Check the cell to the right to see if its connected to a 
        #   known cell
        # x + 1, y
        if x + 1 < self.ogrid_sizeX:
            rc = self.xy2mapIndex(x + 1, y)
            if self.fake_map_data[rc] == -1:
                return True
            elif self.fake_map_data[rc] > 90:
                return connected


        # Check top right
        # x + 1, y + 1
        if x + 1 < self.ogrid_sizeX and y + 1< self.ogrid_sizeY:
            tr = self.xy2mapIndex(x + 1, y + 1)
            if self.fake_map_data[tr] == -1:
                return True
            elif self.fake_map_data[tr] > 90:
                return connected            
        
        # Check bottom right
        # x + 1, y - 1
        if x + 1 < self.ogrid_sizeX and y - 1 >= 0:
            br = self.xy2mapIndex(x + 1, y - 1)
            if self.fake_map_data[br] == -1:
                return True
            elif self.fake_map_data[br] > 90:
                return connected            
        
        
        return connected
    
    
    def Frontier(self):
        """ This funtion finds the frontier on the map and returns a 1D vector 
            containing the indices of the cells on the frontier."""
        frontier = []
        for i in range(len(self.fake_map_data)):
            if self.onFrontier(i):
                frontier.append(i)
        
        return frontier
    
    
    def getFrontier(self):
        """ This function defines and labels the frontier. If the froniter is 
            smaller than a meter it is removed.""" 
        unlabeledFrontier = self.Frontier()
        labels, equiv = self.blogDetection(unlabeledFrontier)        
        print(equiv)
        # Initialize the frontier list
        frontier = []
        frontier_dict = {}
        for n in range(1,max(labels)+1):
            frontier_dict[n] = []
            
        # Second pass to remove equivilencies and store the frontiers
        for i in range(len(labels)):
            frontier_dict.get(labels[i]).append(unlabeledFrontier[i])
        
        print(frontier_dict)
            # Next store the index of the map into the correct row of the
            #   frontier 
        for i in range(len(equiv)):
            rel = equiv[i] # list of equal lables
            j = 0
            while j + 1 < len(rel):
                if type(frontier_dict.get(rel[j])) == list:
                    cur_list = frontier_dict.get(rel[j])
                    equal_lab = rel[j + 1]
                    self.mergeList(frontier_dict,cur_list,equal_lab)
                    frontier_dict[rel[j]] = equal_lab
                j = j + 1

        #print(frontier_dict)
        values = list(frontier_dict.values())
        frontier = [e for e in values if type(e) ==list]
        # Remove all frontiers smaller than 100 cm   
        
        return frontier
    
    def mergeList(self,mydict,cur_list,next):
        if type(mydict.get(next)) == list:
            mydict.get(next).extend(cur_list)
        else:
            self.mergeList(mydict, cur_list, mydict.get(next))

    
    def blogDetection(self, frontier):
        """ This function runs the connected components algorithm on the 
            inputed frontier, which is a 1D vector containing the indices of 
            the cells on the frontier.
        """
        labels = np.zeros_like(frontier, dtype=np.int16)
        full_labels = np.ones_like(self.fake_map_data, dtype=np.int16)*-1
        equiv = []
        cur_label = 0
        cntr = -1
        # Do first pass and label frontiers
        for i in range(len(frontier)):
            #print(frontier[i])
            near_frontier = self.getNearFrontier(frontier[i],frontier)
            near_labels = []
            for e in range(len(near_frontier)):
                ei = frontier.index(near_frontier[e])
                near_labels.append(labels[ei])
            nonzero_labels = [e for e in near_labels if e !=0]
            #print(str(near_labels) +" => " + str(nonzero_labels))
            
            if len(nonzero_labels) == 0:
                cur_label = cur_label + 1
                temp_label = cur_label
            elif len(nonzero_labels) == 1:
                temp_label = nonzero_labels[0]
            else:
                u_l = list(set(nonzero_labels))
                temp_label = max(nonzero_labels)
                if len(u_l) > 1:
                    if nonzero_labels not in equiv:
                        equiv.append(nonzero_labels)
            
            labels[i] = temp_label
            #print("Label " + str(frontier[i]) + " with " + str(temp_label))
                
            
        return labels, equiv
   
    def getNearFrontier(self,index,frontier):
        near_frontier = []
        x,y = self.mapIndex2xy(index)
        
        if x -1 >= 0:
            if self.xy2mapIndex(x -1, y) in frontier:
                near_frontier.append(self.xy2mapIndex(x -1, y))
        
        if x -1 >= 0 and y -1 >= 0:
            if self.xy2mapIndex(x -1, y -1 ) in frontier:
                near_frontier.append(self.xy2mapIndex(x -1, y - 1))
                
        if x -1 >= 0 and y + 1 < self.ogrid_sizeY:
            if self.xy2mapIndex(x -1, y +1 ) in frontier:
                near_frontier.append(self.xy2mapIndex(x -1, y + 1))
        
        if y -1 >= 0:
            if self.xy2mapIndex(x, y -1) in frontier:
                near_frontier.append(self.xy2mapIndex(x, y -1 ))
        
        if y + 1 < self.ogrid_sizeY:
            if self.xy2mapIndex(x, y + 1) in frontier:
                near_frontier.append(self.xy2mapIndex(x, y + 1))
        
        if x + 1 < self.ogrid_sizeX:
            if self.xy2mapIndex(x  + 1, y) in frontier:
                near_frontier.append(self.xy2mapIndex(x + 1, y))
        
        if x +1 < self.ogrid_sizeX and y + 1 < self.ogrid_sizeY:
            if self.xy2mapIndex(x + 1, y + 1) in frontier:
                near_frontier.append(self.xy2mapIndex(x + 1, y+1))
                
        if x + 1 < self.ogrid_sizeX and y -1 >= 0:
            if self.xy2mapIndex(x + 1, y - 1) in frontier:
                near_frontier.append(self.xy2mapIndex(x + 1, y - 1))
        return near_frontier
    
    def testChart(self):
        size = 100
        xs = []
        ys = []
        fxs = []
        fys = []
        
        j = 0
        for i in range(len(self.fake_map_data)):
            if j <= 100:
                self.fake_map_data[i] = 100
                x,y = self.mapIndex2xy(i)
                xs.append(x)
                ys.append(y)
                
            else:
                self.fake_map_data[i] = 0
                x,y = self.mapIndex2xy(i)
                fxs.append(x)
                fys.append(y)
            
            j = j + 1

        
        
        
        matplotlib.rcParams['axes.unicode_minus'] = False
        fig, ax = plt.subplots()
        ax.plot(xs, ys, "ko",fxs,fys,"co")
        ax.set_title('Testing')
        plt.show()        
        
    
    
    def __init__(self):
        self.ogrid_sizeX = grid_size
        self.ogrid_sizeY = grid_size
        self.frontier = [] # index
        self.fake_map_data = [0]*grid_size*grid_size # row-major order
        #self.testChart()
        self.labelFunc()
        pass
    
    
    
    

if __name__ == '__main__':
    t = Test()