from tkinter import *
# from ..class_defs import Cone
import os

## import depanding on running state / configuration state:
from ..config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')


"""
Parse map text to cone array
"""

BLUE = 0
YELLOW = 1
CAR = 2
SPEED = 3

class Vector:
	x = 0
	y = 0
	r = 0
	Type = 0


class Parse_Map2ConeArray:
	def readMap(self,fileName):

		#relative path from master directory:
		try:
			path=os.getcwd()+os.sep+"Code_RealTime"+os.sep+"ConeMapping"+os.sep+fileName
			toRead=open(path, 'r')
		# when current path (cwd) is the correct folder:
		except Exception as ex:
			path=fileName
			toRead=open(path, 'r')
		
		cones = []
		pointStr = toRead.readline()
		while pointStr != '':
			a = pointStr.split(" ")

			#newCone = Cone()
			newCone = Vector()
			newCone.x = int(a[0])
			newCone.y = int(a[1])
			if int(a[2]) == BLUE:
				newCone.Type = messages.perception.Blue
			if int(a[2]) == YELLOW:
				newCone.Type = messages.perception.Yellow
			newCone.type = int(a[2])
			cones.append(newCone)
			pointStr = toRead.readline()

		toRead.close()
		return cones
		

	def __init__(self,fileName):
		self.cones = self.readMap(fileName)

		#change function here
		'''
		orderFunc = OrderByDis(self.cones,carState)
		bluePoints , yellowPoints = orderFunc.orderByDis()
		
		'''
		# bluePoints, yellowPoints, LostBlue, LostYellow = orderByDeluanay( self.cones, self.carPos, self.carDir)

			
	def printMap(self):
		root = Tk()

		my_canvas = Canvas(root, width=1200, height=600)

		for cone in self.cones:
			if cone.type == BLUE:
				my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="blue")
			if cone.type == YELLOW:
				my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="yellow")
			
		my_canvas.pack()

		root.mainloop()

#change the name of the map to be loaded here
cone_array = Parse_Map2ConeArray("Map1.txt")
cone_array.printMap()
