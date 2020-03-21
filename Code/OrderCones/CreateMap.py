from tkinter import *
import math
import numpy

"""
Left click to create yellow cones
Right click to create blue cones
First scroll click creates car
Second scroll click creates hist direction
The map won't be saved if car and direction wern't created
"""


class createMap:
	def __init__(self,fileName):
		self.fileName = fileName
	
	def leftClick(self,event):
		x = event.x
		y = event.y
		self.my_canvas.create_oval(x-3,y-3,x+3,y+3,fill="yellow")
		self.yellowPoints.append([x,y])
		
	def rightClick(self,event):
		x = event.x
		y = event.y
		self.my_canvas.create_oval(x-3,y-3,x+3,y+3,fill="blue")
		self.bluePoints.append([x,y])
		
	def midClick(self,event):
		x = event.x
		y = event.y
		if self.ncar == 0:
			self.my_canvas.create_oval(x-3,y-3,x+3,y+3,fill="purple")
			self.saveX = x
			self.saveY = y
			self.ncar = 1
			self.carPoint.append(x)
			self.carPoint.append(y)
			
		elif self.ncar == 1:
			self.ncar = 2
			norm = math.sqrt((x - self.saveX)**2 + (y - self.saveY)**2) / 40
			self.carDir.append(round((x - self.saveX)/norm))
			self.carDir.append(round((y - self.saveY)/norm))
			self.my_canvas.create_line(self.saveX,self.saveY,self.saveX+self.carDir[0],self.saveY+self.carDir[1],fill="red")

	def close(self):
		if self.ncar == 2:
			toWrite=open(self.fileName, 'w')
			toWrite.write(str(self.carPoint[0]) + " " + str(self.carPoint[1]) + "\n")
			toWrite.write(str(self.carDir[0]) + " " + str(self.carDir[1]) + "\n")
			for point in self.bluePoints:
				dis = str(math.sqrt((point[0]-self.carPoint[0])**2 +(point[1]-self.carPoint[1])**2))
				pos = str(point[0]) + " " + str(point[1])
				toWrite.write( dis + " " + pos + " 1\n")
			for point in self.yellowPoints:
				dis = str(math.sqrt((point[0]-self.carPoint[0])**2 +(point[1]-self.carPoint[1])**2))
				pos = str(point[0]) + " " + str(point[1])
				toWrite.write( dis + " " + pos + " 0\n")
			toWrite.close()
		
		self.root.destroy()
		
	def create(self):		
		self.root = Tk()

		self.my_canvas = Canvas(self.root, width=1200, height=600)
		self.ncar = 0
		self.saveX = 0
		self.saveY = 0
		self.bluePoints = []
		self.yellowPoints = []
		self.carPoint = []
		self.carDir = []
		self.my_canvas.grid(row=0,column=0)
		self.my_canvas.bind("<Button-1>",self.leftClick)
		self.my_canvas.bind("<Button-3>",self.rightClick)
		self.my_canvas.bind("<Button-2>",self.midClick)
		self.closebutton = Button(self.root, text='Save', command=self.close)
		self.my_canvas.pack()
		self.closebutton.pack()
		self.root.mainloop()
		
#change name of the the map to be saved here
cp = createMap("Map1.txt")
cp.create()