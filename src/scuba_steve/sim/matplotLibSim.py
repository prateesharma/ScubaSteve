from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
import numpy as np
from submarine import Submarine
plt.style.use('ggplot')

# Fixing random state for reproducibility
np.random.seed(19680801)
dict1 = {}
sub1 = Submarine(0,0,0)


def randrange(n, vmin, vmax):
    '''
    Helper function to make an array of random numbers having shape (n, )
    with each number distributed Uniform(vmin, vmax).
    '''
    return (vmax - vmin)*np.random.rand(n) + vmin


#takes in x,y,and z parameter of the grid
def createPlot(x,y,z):
    def traverseGrid(x, y, z, grid,plt,ax):
        point1 = ax.scatter(x, y, z, c='g', marker='o')
        print(x,y,z)
        # point1.set_visible(False)
        plt.pause(.1)

        plt.show()


        # base case
        print(grid)
        if x ==grid.shape[0]-1 and y == grid.shape[0]-1 and z == grid.shape[0]-1:
            return True

        if validPoint(x, y, z, grid):
            grid[x][y][z] = -1
            point2 = ax.scatter(x,y,z, c='b', marker = 'o')

            if traverseGrid(x, y + 1, z, grid,plt,ax):
                return True
            if traverseGrid(x + 1, y, z, grid,plt,ax):
                return True

            if x>0:
                if traverseGrid(x - 1, y, z, grid,plt,ax):
                    return True
                else:
                    return False

            if y>0:

                if traverseGrid(x, y - 1, z, grid,plt,ax):
                    return True
                else:
                    return False

            if traverseGrid(x, y, z + 1, grid,plt,ax):
                return True

            if z>0:
                if traverseGrid(x, y, z - 1, grid,plt,ax):
                    return True
                else:
                    return False
            # if none of these work then we backtrack

            grid[x][y][z] = 0


        return False

    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #number of points to be added
    n = 2

    #setting random points
    xs = randrange(n, 0, x)
    ys = randrange(n, 0, y)
    zs = randrange(n, 0, z)
    ax.scatter(2, 2, 2, c='r', marker='o')

    #add red dots to a dictionary
    list1 = addToList(xs,ys,zs)

    # #set x,y,and z points for the sub
    # sub1xs = randrange(1,0,x)
    # sub1ys = randrange(1,0,y)
    # sub1zs = randrange(1,-z,0)
    #
    # #set the variables for the sub object
    # sub1.x = sub1xs
    # sub1.y = sub1ys
    # sub1.z = sub1zs


    #setting axis labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')


    grid = createGrid(x,y,z)


    if traverseGrid(0,0,0,grid,plt,ax):
        return sub1

def validPoint(x,y,z,grid):
    print(x,y,z,grid)
    if x<0 or y<0 or z<0 or x>=grid.shape[0] or y>=grid.shape[1] or z>=grid.shape[2] or grid[x][y][z] == -1:
        return False
    return True


def createGrid(x,y,z):
    grid = np.zeros((x,y,z))
    print(grid.shape)
    return grid


def addToList(xs,ys,zs):
    list1 = []

    for i in range(len(xs)):
        list1.append([int(xs[i]),int(ys[i]),int(zs[i])])
    return list1

if __name__ == '__main__':
    createPlot(10,10,10)
