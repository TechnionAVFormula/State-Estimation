import math
## For relative path:
import sys
import os
from pathlib import Path
current_path  = os.path.dirname(__file__)
current_path  = Path(current_path)
relative_path = current_path.parent
sys.path.append(str(relative_path))

from OrderCones.orderByDistance import OrderByDis
from OrderCones.OrderByDeluanay import orderByDeluanay
import tkinter as tk


OrderByDeluanayParams = dict(MaxIterations = 25 )


def orderCones(coneMap, carState  ):
	'''
	this function takes the map and trys cone ordering algorithims untill one is succesful
	Input:
    coneMap - iterable group of cones, where each cone contains position.x, position.y and type
    CarState - list of car properties containting CarState.x, CarState.y, and theta
	'''
	
	if all( coneMap==None ):
		return list(), list()

	#first algorithim - delaunay
	bluePoints , yellowPoints , blueLost ,yellowLost, successStatus = orderByDeluanay(coneMap, carState, OrderByDeluanayParams)
	if successStatus == 1:
		return bluePoints , yellowPoints

	#last algorithim - distance
	#never fails
	orderClass = OrderByDis(coneMap , carState)
	bluePoints , yellowPoints = orderClass.orderByDis()
	return bluePoints , yellowPoints
	
def printMap(bluePoints , yellowPoints , blueLost ,yellowLost, carState):
	root = tk.Tk()

	my_canvas = tk.Canvas(root, width=1200, height=600)

	order = 1
	for cone in bluePoints:
		my_canvas.create_oval(200+5*cone.position.x-3,200+5*cone.position.y-3,200+5*cone.position.x+3,200+5*cone.position.y+3,fill="blue")
		label = tk.Label(master = root,text=str(order))
		order += 1
		label.place(x=200+5*cone.position.x-3 - 5,y= 200+5*cone.position.y - 30)
	for cone in blueLost:
		my_canvas.create_oval(200+5*cone.position.x-3,200+5*cone.position.y-3,200+5*cone.position.x+3,200+5*cone.position.y+3,fill="blue")
		label = tk.Label(master = root,text=str(0))
		label.place(x=200+5*cone.position.x - 5,y= 200+5*cone.position.y - 30)
	order = 1
	for cone in yellowPoints:
		my_canvas.create_oval(200+5*cone.position.x-3,200+5*cone.position.y-3,200+5*cone.position.x+3,200+5*cone.position.y+3,fill="yellow")
		label = tk.Label(master = root,text=str(order))
		order += 1
		label.place(x=200+5*cone.position.x - 5,y= 200+5*cone.position.y - 30)
	for cone in yellowLost:
		my_canvas.create_oval(200+5*cone.position.x-3,200+5*cone.position.y-3,200+5*cone.position.x+3,200+5*cone.position.y+3,fill="yellow")
		label = tk.Label(master = root,text=str(0))
		label.place(x=200+5*cone.position.x - 5,y= 200+5*cone.position.y - 30)

	carState.theta = math.pi/2
	numVel = [10*math.cos(carState.theta) ,10*math.sin(carState.theta)]

	my_canvas.create_oval(200+5*carState.position.x-3,200+5*carState.position.y-3,200+5*carState.position.x+3,200+5*carState.position.y+3,fill="purple")
	my_canvas.create_line(200+5*carState.position.x,200+5*carState.position.y,200+5*carState.position.x+numVel[0],200+5*carState.position.y+numVel[1],fill="red")
	my_canvas.pack()

	root.mainloop()