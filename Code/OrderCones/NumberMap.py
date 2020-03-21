from orderByDistance import *
from tkinter import *

"""
Take saved map and order cones by chosen function
"""

BLUE = 0
YELLOW = 1
CAR = 2
SPEED = 3

class Vector:
	def __init__(self, x, y, type):
		self.x = x
		self.y = y
		self.type = type

class NumberdMap:
	def readMap(self,fileName):
		toRead=open(fileName, 'r')

		carPos = Vector(int(toRead.readline()),int(toRead.readline()),CAR)
		carDir = Vector(int(toRead.readline()),int(toRead.readline()),SPEED)

		cones = []
		pointStr = toRead.readline()
		while pointStr != "break\n":
			a = pointStr.split(" ")
			newCone = Vector(int(a[0]),int(a[1]), BLUE)
			cones.append(newCone)
			pointStr = toRead.readline()
			
		pointStr = toRead.readline()
		while pointStr != '':
			a = pointStr.split(" ")
			newCone = Vector(int(a[0]),int(a[1]), YELLOW)
			cones.append(newCone)
			pointStr = toRead.readline()

		toRead.close()
		return carPos, carDir, cones
		

	def __init__(self,fileName):
		self.carPos, self.carDir, cones = self.readMap(fileName)
		
		#change function here
		self.cones = orderByDis(cones,self.carPos,self.carDir)
			
	def printMap(self):
		root = Tk()

		my_canvas = Canvas(root, width=1200, height=600)

		for cone in self.cones:
			if cone.type == BLUE:
				my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="blue")
				label = Label(master = root,text=str(cone.order))
				label.place(x=cone.x - 5,y= cone.y - 30)
			if cone.type == YELLOW:
				my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="yellow")
				label = Label(master = root,text=str(cone.order))
				label.place(x=cone.x - 5,y= cone.y - 30)

		my_canvas.create_oval(self.carPos.x-3,self.carPos.y-3,self.carPos.x+3,self.carPos.y+3,fill="purple")
		my_canvas.create_line(self.carPos.x,self.carPos.y,self.carPos.x+self.carDir.x,self.carPos.y+self.carDir.y,fill="red")
			
		my_canvas.pack()

		root.mainloop()

#change the name of the map to be loaded here
np = NumberdMap("Map1.txt")
np.printMap()
