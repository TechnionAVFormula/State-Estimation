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



def orderCones(coneMap, carState  ):
	#to order cones by distance

	#orderClass = OrderByDis(coneMap , carState)
	#bluePoints , yellowPoints = orderClass.orderByDis()
	#printMap(bluePoints, yellowPoints, [],[],carState)


	# carState.theta = math.pi/2
	#to order cones by delaunay
	bluePoints , yellowPoints , blueLost ,yellowLost = orderByDeluanay(coneMap, carState)
	printMap(bluePoints , yellowPoints , blueLost ,yellowLost, carState)
	return bluePoints  ,yellowPoints
	
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
    

