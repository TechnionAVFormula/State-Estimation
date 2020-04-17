from .OrderByDistance import OrderByDis
from .OrderByDeluanay import orderByDeluanay
import tkinter as tk



def orderCones(coneMap, carState  ):
	#to order cones by distance
	'''
    orderClass = OrderByDis(coneMap , carState)
    bluePoints , yellowPoints = orderClass.orderByDis()
	'''
	
	#to order cones by delaunay
	bluePoints , yellowPoints , blueLost ,yellowLost = orderByDeluanay(coneMap, carState)
	
	return bluePoints  ,yellowPoints
	
def printMap(bluePoints , yellowPoints , blueLost ,yellowLost, carState):
	root = tk.Tk()

	my_canvas = tk.Canvas(root, width=1200, height=600)

	for cone in bluePoints:
		my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="blue")
		label = tk.Label(master = root,text=str(cone.order))
		label.place(x=cone.x - 5,y= cone.y - 30)
	for cone in blueLost:
		my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="blue")
		label = tk.Label(master = root,text=str(cone.order))
		label.place(x=cone.x - 5,y= cone.y - 30)
	for cone in yellowPoints:
		my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="yellow")
		label = tk.Label(master = root,text=str(cone.order))
		label.place(x=cone.x - 5,y= cone.y - 30)
	for cone in yellowLost:
		my_canvas.create_oval(cone.x-3,cone.y-3,cone.x+3,cone.y+3,fill="yellow")
		label = tk.Label(master = root,text=str(cone.order))
		label.place(x=cone.x - 5,y= cone.y - 30)

	my_canvas.create_oval(carState.x-3,carState.y-3,carState.x+3,carState.y+3,fill="purple")
	my_canvas.create_line(carState.x,carState.y,carState.x+carState.Vx,carState.y+carState.Vy,fill="red")
	
	my_canvas.pack()

	root.mainloop()
    

