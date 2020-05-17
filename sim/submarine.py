class Submarine:
    def __init__(self,x,y,z):
        #create a submarine object that has x,y and z coordinates
        self.x = x
        self.y = y
        self.z = z
        #targets is a list storing coordinates of target objects
        self.targets = []

    def moveForwardX(self):
        self.x +=1

    def moveBackwardX(self):
        self.x -=1

    def moveForwardY(self):
        self.y +=1

    def moveBackwardY(self):
        self.y -=1

    def moveForwardZ(self):
        self.z +=1

    def moveBackwardZ(self):
        self.z -=1



