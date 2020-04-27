import math
import numpy as np
from scipy.spatial import Delaunay
import copy

## for relative path
import os 
import sys
current_dir_name = os.path.dirname(__file__)
relative_dir_name = os.path.join(current_dir_name, '..')
sys.path.append(relative_dir_name)


## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')

# Get proper Enum:
YELLOW = messages.perception.Yellow
BLUE = messages.perception.Blue
ORANGE = messages.perception.OrangeBig
ORANGE_SMALL = messages.perception.OrangeSmall

FAKECONE = -10

'''
orderByDeluanay takes:
CarState - contains x,y,Vx,Vy
Cones- array of cones in some format

orderByDeluanay returns:
returnBlue- array of ordered blue cones in same format as input
returnYellow - array of ordered yellow cones in same format as input
returnLostBlue - array of blue cones not chosen to be ordered in same format as input
returnLostYellow - array of yellow cones not chosen to be ordered in same format as input
'''

def orderByDeluanay(Cones, CarState):
	nCones = len(Cones)
	coneTemplate = copy.deepcopy(Cones[0])
	numCones = np.empty([len(Cones), 4])
	numCar = [CarState.position.x ,CarState.position.y]
	numVel = [CarState.velocity.x ,CarState.velocity.y]
	for i in range(nCones):
		color = 0
		if Cones[i].type == YELLOW:
			numCones[i][0], numCones[i][1], numCones[i][2] = Cones[i].position.x, Cones[i].position.y, 121
			numCones[i][3] = i
		elif Cones[i].type == BLUE:
			numCones[i][0], numCones[i][1], numCones[i][2] = Cones[i].position.x, Cones[i].position.y, 98
			numCones[i][3] = i

	mt = MapTrack(numCones, numCar, numVel)
	mt.OrderCones()
	bCones, yCones = mt.OrderedBlueCones, mt.OrderedYellowCones
	bLostCones, yLostCones = mt.LostBlue, mt.LostYellow

	order = 1
	returnYellow = []
	hasAppeard = np.zeros(nCones)
	for cone in yCones:
		if cone[3] != FAKECONE and hasAppeard[int(cone[3])] != 1:
			hasAppeard[int(cone[3])] = 1
			returnCone = copy.deepcopy(coneTemplate)
			returnCone.position.x = cone[0]
			returnCone.position.y = cone[1]
			returnCone.type = YELLOW
			#returnCone.order = order
			returnYellow.append(returnCone)
			order += 1
	
	order = 1
	returnBlue = []
	for cone in bCones:
		if cone[3] != FAKECONE and hasAppeard[int(cone[3])] != 1:
			hasAppeard[int(cone[3])] = 1
			returnCone = copy.deepcopy(coneTemplate)
			returnCone.position.x = cone[0]
			returnCone.position.y = cone[1]
			returnCone.type = BLUE
			#returnCone.order = order
			returnBlue.append(returnCone)
			order += 1	

	returnLostBlue = []
	for cone in bLostCones:
		if cone[3] != FAKECONE and cone[2] != 0:
			returnCone = copy.deepcopy(coneTemplate)
			returnCone.position.x = cone[0]
			returnCone.position.y = cone[1]
			returnCone.type = BLUE
			#returnCone.order = 0
			returnLostBlue.append(returnCone)
		
	returnLostYellow = []
	for cone in yLostCones:
		if cone[3] != FAKECONE and cone[2] != 0:
			returnCone = copy.deepcopy(coneTemplate)
			returnCone.position.x = cone[0]
			returnCone.position.y = cone[1]
			returnCone.type = YELLOW
			#returnCone.order = 0
			returnLostYellow.append(returnCone)

	for cone in mt.BackCones:
		if cone[3] != FAKECONE:
			returnCone = copy.deepcopy(coneTemplate)
			returnCone.position.x = cone[0]
			returnCone.position.y = cone[1]
			#returnCone.order = -1
			if cone[2] == 98:
				returnCone.type = BLUE
				returnLostBlue.append(returnCone)
			elif cone[2] == 121:
				returnCone.type = YELLOW
				returnLostYellow.append(returnCone)
		
		
	return returnBlue, returnYellow, returnLostBlue, returnLostYellow
	
def getWeights(Cones, CarState):
	coneTemplate = copy.deepcopy(Cones[0])
	numCones = np.empty([len(Cones), 4])
	numCar = [CarState.x,CarState.y]
	numVel = [CarState.Vx,CarState.Vy]
	for i in range(len(Cones)):
		color = 0
		if Cones[i].type == YELLOW:
			numCones[i][0], numCones[i][1], numCones[i][2] = Cones[i].x, Cones[i].y, 121
			numCones[i][3] = i
		elif Cones[i].type == BLUE:
			numCones[i][0], numCones[i][1], numCones[i][2] = Cones[i].x, Cones[i].y, 98
			numCones[i][3] = i
	
	mt = MapTrack(numCones, numCar, numVel)
	return mt.getWeights()


class MapTrack:
	def __init__(self,Cones,CarCG,CarVel,SphereR=10000, CarLength=1):
		self.Cones=Cones #matrix mx3 [x,y,color]. color = 98(blue)/121(yellow)
		self.CarCG=CarCG #[x,y]
		self.CarDir=CarVel/np.linalg.norm(CarVel) #[x,y] of car direction

		#Geomtetric parameters
		self.SphereR=SphereR #10 is an expiramental number
		self.CarLength=CarLength#0.1 is an expiramental number
		
		#save a copy of the cones before processing
		self.SaveCones = copy.deepcopy(self.Cones)

		#initalize output of algorithm
		self.Cones4Calc=self.FindCones4Calc() #cones from which track is calculated.
		self.OrderedBlueCones=np.array([]) #Calculated in OrderCones
		self.OrderedYellowCones=np.array([]) #Calculated in OrderCones
		self.MidPoints=np.array([]) #Calculated in FindMidPoints

		self.LostBlue = []
		self.LostYellow = []
		
	def FindCones4Calc(self,Angle4FakeCones=np.pi/3):
		#Preprocessing
		Cones=SphereFilter(self.Cones,self.CarCG,self.SphereR)[0] #filtering for radius around car

		#Fix new points - fake cones - if no blue/yellow cones are found
		BlueCones=Cones[(Cones[:,2]==98),:]
		FilteredBlueCones=BackFilter(BlueCones, self.CarCG, self.CarDir)
		FrontBlueCones = FilteredBlueCones[0]
		BackBlueCones = FilteredBlueCones[1]
		
		YellowCones = Cones[(Cones[:, 2] == 121), :]
		FilteredYellowCones = BackFilter(YellowCones, self.CarCG, self.CarDir)
		FrontYellowCones = FilteredYellowCones[0]
		BackYellowCones = FilteredYellowCones[1]
		
		self.BackCones = np.vstack( (BackYellowCones, BackBlueCones) )
		
		BaddFlag=FrontBlueCones.size==0
		YaddFlag=FrontYellowCones.size==0
		if BaddFlag:
			BNew=self.CarCG+self.CarLength*RotateVector(self.CarDir,+Angle4FakeCones) #add cone to the left
			Cones=np.vstack([Cones,np.hstack([BNew,98,FAKECONE])]) #add fake yellow cone @ the buttom of matrix
		if YaddFlag: #no yellow cones exist
			YNew=self.CarCG+self.CarLength*RotateVector(self.CarDir,-Angle4FakeCones) #add cone to the right
			Cones=np.vstack([Cones,np.hstack([YNew,121,FAKECONE])]) #add fake yellow cone @ the buttom of matrix

		self.Cones4Calc=Cones
		return Cones
	
	#for testing purposes
	def getWeights(self):
		if self.Cones4Calc.shape[0] < 3:
			return 0
			
		#initalize
		Cones=self.Cones4Calc
		
		points = Cones[:,:2]
		
		#Triangulate
		DT=Delaunay(points) #Triangulate only /w Cones
		return (points[DT.simplices])
		
	def OrderCones(self,MaxItrAmnt=25,CostThreshold=-0.2,ColorCostWeight=0.3,
                   RRatioThreshold=20):
		'''
		Input:
		self

		Output:
		to class:
		BCones - blue cones in order for interpolation [x,y]
		Ycones - yellow cones in order for interpolation [x,y]
		'''
		if self.Cones4Calc.shape[0] < 3:
			return 0
			
		#initalize
		Yind=np.full([MaxItrAmnt,1],np.nan)
		Bind=np.full([MaxItrAmnt,1],np.nan)
		Cones=self.Cones4Calc
		
		#Triangulate
		DT=Delaunay(Cones[:,:2]) #Triangulate only /w Cones
		ID=DT.find_simplex(self.CarCG) #attempt to find triangle which contains CarCG
		if ID==-1: #if CarCG isoutside of convex hull
			Cones=np.vstack([Cones,np.hstack([self.CarCG,0,FAKECONE])]) #add Car to Cones as a fake cone
			DT=Delaunay(Cones[:,:2])#Triangulate /w Cones+CarCG
			ID=DT.find_simplex(self.CarCG+0.5*self.CarLength*self.CarDir) #find first triangle to work with
		if ID == -1: return  # no cones in sight. couldnt build a track

		#Important note: DT.find_simplex returns -1 if point is outside triangulation.
		#But we use setdiff1d in both TriOne and FindNextTriangle so NewID can be of empty.
		NewID,Newu,NewCrossEdge=TriOne(DT,Cones[:,2],ID,self.CarDir,ColorCostWeight)
		if NewID.size==0: return #crossed into no-man's land

		#insert first cones into lists (Bind/Yind)
		if Cones[NewCrossEdge[0],2]==ord('y'):
			Yind[0]=NewCrossEdge[0]
		else:
			Bind[0]=NewCrossEdge[0]
		if Cones[NewCrossEdge[1],2]==ord('y'):
			Yind[0]=NewCrossEdge[1]
		else:
			Bind[0]=NewCrossEdge[1]
		NewV=np.setdiff1d(DT.simplices[NewID,:],NewCrossEdge)
		if Cones[NewV,2]==ord('y'):
			Yind[1]=NewV
		else:
			Bind[1]=NewV
			
			
		for Itr in range(2,MaxItrAmnt):
			#find next triangle
			NewID,Newu,NewCrossEdge,Cost,RRatio=FindNextTriangle(DT,Cones[:,2],NewID,Newu,NewCrossEdge,ColorCostWeight)
			#check conditions, if not good enough - break
			if NewID.size==0 or CostThreshold<Cost or RRatioThreshold<RRatio:
				break
			#add next cone to Bind/Yind
			NewV=np.setdiff1d(DT.simplices[NewID,:],NewCrossEdge)
			if Cones[NewV,2]==ord('y'):
				Yind[Itr]=NewV
			else:
				Bind[Itr]=NewV

		#create output
		Bind=Bind[~np.isnan(Bind)].astype(int)
		Yind=Yind[~np.isnan(Yind)].astype(int) #delete rows with NaNs
		BCones=Cones[Bind,:4]
		YCones=Cones[Yind,:4]

		#Output to class
		self.OrderedBlueCones=BCones
		self.OrderedYellowCones=YCones
		
		#Output Filtered Cones to class
		self.SetFilteredCones()
		
	def SetFilteredCones(self):
		isFiltered = np.zeros(int((self.SaveCones.size)/4))
		
		nBlueCones = 0
		nYellowCones = 0
		nBackBlueCones = 0
		nBackYellowCones = 0
		for cone in self.SaveCones:
			if cone[2] == 98:
				nBlueCones +=1
			elif cone[2] == 121:
				nYellowCones +=1
				
		for cone in self.OrderedBlueCones:
			isFiltered[int(cone[3])] = 1
		for cone in self.OrderedYellowCones:
			isFiltered[int(cone[3])] = 1
		for cone in self.BackCones:
			if isFiltered[int(cone[3])] != 1:
				isFiltered[int(cone[3])] = 2
				if cone[2] == 98:
					nBackBlueCones += 1
				if cone[2] == 121:
					nBackYellowCones += 1
			else:
				cone[2] = 0
				
		nFilteredBlueCones = int(self.OrderedBlueCones.size/4)
		nFilteredYellowCones = int(self.OrderedYellowCones.size/4)
		self.LostBlue = np.zeros([nBlueCones - nFilteredBlueCones - nBackBlueCones + 1, 4])
		self.LostYellow = np.zeros([nYellowCones - nFilteredYellowCones - nBackYellowCones + 1, 4])
		bCounter = 0
		yCounter = 0
		for i in range(0, isFiltered.size):
			if int(isFiltered[i]) == 0 and self.SaveCones[i][3] != FAKECONE:
				if int(self.SaveCones[i][2]) == 98:
					self.LostBlue[bCounter] = self.SaveCones[i]
					bCounter += 1
				elif int(self.SaveCones[i][2]) == 121:
					self.LostYellow[yCounter] = self.SaveCones[i]
					yCounter += 1
		
		
		
#Associated functions with class ConesDT
def TriOne(DT,ConesColors,ID,CarDir,ColorCostWeight):
	'''
	Input:
	DT - DelaunayTriangulation containing DT.points and DT.simplices
	ConesColors - colors of cones (98 for blue, 121 for yellow) with same
	indexing as DT.Points
	ID - number of triangle in DT.simplices (row index)
	CarDir - car direction [x,y] normalized
	ColorCostWeight - weight for costfunction in triangles. Applied on first term - effect cone colors.

	Output:
	NewID - number of new triangle in DT.ConnectivityList (row index)
	Newu - direction of enterance to new triangle (NewID)
	NewCrossEdge - [V1,V2] of edge that we crossed to get from ID->NewID
	V1 and V2 refer to vertex indcies (row) of DT.points
	'''
	V=DT.simplices[ID,:] #find vertex indcies of ID
	P=np.hstack([(DT.points[V,:]),ConesColors[V].reshape(3,1)]) #Triangle cones
	IC=InCenter(P[:,:2])[0] #find incenter of ID
	EdgesInd=np.array([[0,1], #Edge1 #build indcies in V mapping to EdgesV by V[EdgesInd]
					   [0,2], #Edge2
					   [1,2]])#Edge3
	J1=EdgeCost(IC,P[EdgesInd[0,:],:],CarDir,ColorCostWeight)
	J2=EdgeCost(IC,P[EdgesInd[1,:],:],CarDir,ColorCostWeight)
	J3=EdgeCost(IC,P[EdgesInd[2,:],:],CarDir,ColorCostWeight)
	J=np.hstack([J1,J2,J3]); JminInd=np.argmin(J)
	NewCrossEdge=V[EdgesInd[JminInd,:]] #obtain Edge Vertices to cross (point indcies [V1,V2])
	Newu=OutFacingNormal(P[EdgesInd[JminInd,:],:2],IC) #calculate normal to cross edge
	Attachments=np.where(np.any(DT.simplices==NewCrossEdge[0],axis=1) &\
						 np.any(DT.simplices==NewCrossEdge[1],axis=1)) #find IDs that are connected to edge
	NewID=np.squeeze(np.setdiff1d(Attachments,ID)) #NewID - New Triangle
	return NewID,Newu,NewCrossEdge
	
def RecursiveEdgeCost(V,P,TriInCenter,edge,edges,Dir,delaunay,level,ColorCostWeight,maxlevel = 5):
	if level == maxlevel:
		return EdgeCost(edge)

	#J2=EdgeCost(TriInCenter,P[edges[1,:],:],Dir,ColorCostWeight)
	#print (J2)
	
	#get next triangle for edge1
	edge1dir = OutFacingNormal(P[edges[0,:],:2],TriInCenter)
	
	
	#get next triangle for edge2
	
	'''edge1cost=RecursiveEdgeCost(edge1,edge11,edge12,edge1dir,delaunay,level+1,maxlevel)
	edge2cost=RecursiveEdgeCost(edge1,edge11,edge12,edge1dir,delaunay,level+1,maxlevel)
	CurrentEdge = (CurrentEdge + min([edge1cost,edge2cost]))/2
		
	return CurrentEdge'''
	return 0

def FindNextTriangle(DT,ConesColors,ID,Dir,CrossEdge,ColorCostWeight):
	'''
	Input:
	DT - DelaunayTriangulation containing DT.points and DT.simplices
	ConesColors - colors of cones (98 for blue, 121 for yellow) with same
	indexing as DT.Points
	ID - number of triangle in DT.ConnectivityList (row index)
	Dir - direction of enterance to triangle ID
	CrossEdge - [V1,V2] of edge that was crossed to enter triangle ID
	V1 and V2 refer to vertex indcies (row) in DT.Points
	ColorCostWeight - weight for costfunction in triangles. Applied on first term - effect cone colors.

	Output:
	NewID - number of new triangle in DT.ConnectivityList (row index)
	NewDir - direction of enterance to new triangle (NewID)
	NewCrossEdge - [V1,V2] of edge that we crossed to get from ID->NewID
	V1 and V2 refer to vertex indcies (row) in DT.Points
	MinCost - price in cost function with which algorithm decided to go
	RRatio - ratio between Circumcenter/InCenter radii
	'''
	V=DT.simplices[ID,:] #find vertex indcies of ID
	P=np.hstack([DT.points[V,:],ConesColors[V].reshape(3,1)]) #Triangle cones
	IC=InCenter(P[:,:2])[0] #find incenter of ID
	EdgesInd=np.array([[0,1], #Edge1 #build indcies in V mapping to EdgesV by V[EdgesInd]
					   [0,2], #Edge2
					   [1,2]])#Edge3
	EdgesV=V[EdgesInd]
	EdgesInd=EdgesInd[~(np.any(EdgesV==CrossEdge[0],axis=1) &\
						np.any(EdgesV==CrossEdge[1],axis=1)),:] #find the two edges to calculate for (not going backwards).
	J1=EdgeCost(IC,P[EdgesInd[0,:],:],Dir,ColorCostWeight) #calculate costs
	J2=EdgeCost(IC,P[EdgesInd[1,:],:],Dir,ColorCostWeight)
	RecursiveEdgeCost(V,P,IC,CrossEdge,EdgesInd,Dir,DT,0,ColorCostWeight)
	J=np.hstack([J1,J2]); MinCost=np.min(J); JminInd=np.argmin(J) #find Edge to cross by row index in Edges
	NewCrossEdge=V[EdgesInd[JminInd,:]] #obtain Edge Vertices to cross (point indcies [V1,V2])
	NewDir=OutFacingNormal(P[EdgesInd[JminInd,:],:2],IC) #calculate normal to cross edge
	Attachments=np.where(np.any(DT.simplices==NewCrossEdge[0],axis=1) &\
						 np.any(DT.simplices==NewCrossEdge[1],axis=1)) #find IDs that are connected to edge
	NewID=np.squeeze(np.setdiff1d(Attachments,ID)) #NewID - New Triangle

	if NewID.size==0:
		RRatio=0 #filler
		return NewID,NewDir,NewCrossEdge,MinCost,RRatio

	NewV=DT.simplices[NewID,:] #find vertex indcies of NewID (=NewTriangle)
	#    NewV=NewV.reshape(3)
	NewP=DT.points[NewV,:] #find points correlating to new vertex indcies
	RRatio=CircumRadius(NewP)/InCenter(NewP)[1] #calculate RRatio of new triangle
	return NewID,NewDir,NewCrossEdge,MinCost,RRatio
	

def EdgeCost(TriInCenter,EdgeCones,u,ColorCostWeight):
	'''
	Input:
	InCenter - [x,y] in center of triangle
	EdgeCones - [x,y,color] 2x3 of edge vertcies
	u - [x,y] normalized direction of enterance to triangle
	ColorCostWeight - weight for different color cones cost. should range [0,1]

	Output:
	J - cost of passing through edge. |J|<2

	Example:
	t1=np.radians([-30,90,210])
	ConesColors=np.array([98,121,98])
	TriCones=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1)),ConesColors])) #equilateral triangle
	EdgeCones=TriCones[:2,:] #edge cones have different colors
	InCenter=InCenter(TriCones[:,:2])[0] #[0] after function call - returns the  first value
	u=OutFacingNormal(EdgeCones[:,:2],InCenter) #enterance direction - same direction as out facing normal of edge
	CostColorWeight=1
	J=EdgeCost(InCenter,EdgeCones,u,alpha)
	print(J) #expect to be -2
	'''
	center = np.array([(EdgeCones[0][0]+EdgeCones[1][0])/2,(EdgeCones[0][1]+EdgeCones[1][1])/2])
	dir1 = np.array([center[0]-EdgeCones[0][0],center[1]-EdgeCones[0][1]])
	if np.cross(u,dir1) > 0:
		leftPoint = EdgeCones[0]
		rightPoint= EdgeCones[1]
	else:
		leftPoint = EdgeCones[1]
		rightPoint= EdgeCones[0]
	J=0 #initalize
	if EdgeCones[0,2]==EdgeCones[1,2]: #if the same color
		J=J+1
	elif leftPoint[2] == 121:
		J=J+1.9
	else: #not the same color
		J=J-1
	J=ColorCostWeight*J-np.dot(u,OutFacingNormal(EdgeCones[:,:2],TriInCenter)) #+bad points for difference in direction
	return J
def BackFilter(Cones,CarCG,CarDir):
	'''
	Input:
	Cones - mx3 cones matrix containing [x,y,color]
	CarCG - [x,y] of car position
	CarDir - [x,y] normalized vector of car direction

	Output:
	FilteredCones - same format as cones. only cones that are infront of the vehicle
	FInd - bool array FilteredCones=Cones(FInd,:)
	'''
	ConesR=Cones[:,:2]-CarCG #vectors of cones relative to car
	DotProducts = np.matmul(ConesR, np.transpose(CarDir))  # dot product with normal (car direction)
	FInd = DotProducts > 0  # find MidPoints infront of the vehicle (index)
	Flost = DotProducts <= 0
	FilteredCones=Cones[FInd,:]
	LostCones = Cones[Flost,:]
	return FilteredCones,LostCones,FInd,Flost
def SphereFilter(Cones,CarCG,R):
	'''
	Input:
	Cones - mx3 cones matrix containing [x,y,color]
	CarCG - [x,y] of car position
	R - radius of hemisphere

	Output:
	FilteredCones - same format as cones
	only cones within sphere with radius R starting at car
	FInd - bool array FilteredCones=Cones(FInd,:)

	Example:
	R=np.sqrt(2)
	Cones=np.array([[-1,0.5,98],
					[1,1,121],
					[1,-0.6,98]])
	CarCG=np.array([0,0.5])
	CarDir=np.array([0,1])
	SphereFilter(Cones,CarCG,R)
	PassCones, FInd=SphereFilter(Cones,CarCG,R) #run function on data
	FailCones=Cones[~FInd,:] #obtain fail cones
	t=np.linspace(0,2*np.pi,100) #for circle plotting
	plt.scatter(PassCones[:,0],PassCones[:,1],color=[0,1,0]) #plot pass cones
	plt.scatter(FailCones[:,0],FailCones[:,1],color=[1,0,0]) #plot fail cones
	plt.scatter(CarCG[0],CarCG[1],color=[0.5,0,0.5]) #plot CarCG
	plt.plot(CarCG[0]+R*np.cos(t),CarCG[1]+R*np.sin(t),ls='--',lw=2,color=[0,0,0]) #plot circle
	plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid
	'''
	ConesR=Cones[:,:2]-CarCG #vectors of cones relative to car
	SqDistance=np.diag(np.matmul(ConesR,np.transpose(ConesR)))
	FInd=SqDistance < R**2
	FilteredCones=Cones[FInd,:]
	return FilteredCones,FInd

#Some basic geometrey functions we need
def ProjPnt2Line(P,q):
	'''
	Input:
	q - [x,y] point to be project onto line P
	P - 2x2 [[x1,y1], points representing a line
			[x2,y2]]

	Output:
	projq  [x,y] np array representing point resulting in projecting q on P

	Example:
	P=np.array([[0,0],
			  [1,0]])
	q=np.array([0.5,4])
	projq=ProjPnt2Line(P,q)
	plt.scatter(P[:,0],P[:,1],color=[1,0,0])
	plt.scatter(q[0],q[1],color=[0,0,0])
	plt.scatter(projq[0],projq[1],color=[0,0,1])
	'''
	p1=P[0,:]; p2=P[1,:]
	t=(p2-p1)/np.linalg.norm(p2-p1) #unit vector in p1-p2
	projq=np.dot((q-p1),t)*t+p1; #find project point
	return projq
def InCenter(P):
	'''
	Input:
	P - 3x2 [[x1,y1], points of a triangle
			 [x2,y2],
			 [x3,y3]]

	Output:
	TriInCenter - [x,y] coordinates
	InR - radius of InCircle

	From:
	https://www.mathopenref.com/coordincenter.html
	https://keisan.casio.com/

	Example:
	t1=np.radians([-30,90,210])
	P=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1))])) #equilateral triangle
	InCenter, InR=InCenter(P)
	plt.plot(np.hstack([P[:,0],P[0,0]]),np.hstack([P[:,1],P[0,1]]),color=[0.5,0,0]) #plot triangle edges
	plt.scatter(P[:,0],P[:,1],color=[1,0,0]) #plot triangle points
	plt.scatter(InCenter[0],InCenter[1],color=[0,0,1]) #plot InCenter
	t2=np.linspace(0,2*np.pi,100) #for circle plotting
	plt.plot(InCenter[0]+InR*np.cos(t2),InCenter[1]+InR*np.sin(t2),ls='--',lw=2,color=[0,0,0]) #plot InCircle
	q=InCenter+InR*(P[0,:]-InCenter)/np.linalg.norm((P[0,:]-InCenter)) #find projection of point 1 on InCircle
	plt.plot([InCenter[0],q[0]],[InCenter[1],q[1]],lw=2,color=[0,0.7,0]) #plot in radius in the direction of point 1
	plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid
	'''
	A=P[0,:]; B=P[1,:]; C=P[2,:]
	a=np.linalg.norm(B-C); b=np.linalg.norm(A-C); c=np.linalg.norm(A-B);
	p=a+b+c
	TriInCenter=(a*A+b*B+c*C)/p
	s=p/2
	InR=np.sqrt(s*(s-a)*(s-b)*(s-c))/s
	return TriInCenter,InR
def CircumRadius(P):
	'''
	Input:
	P - 3x2 [[x1,y1], points of a triangle
			 [x2,y2],
			 [x3,y3]]

	Output:
	CircumR=radius of CircumCircle

	From:
	http://mathworld.wolfram.com/Circumradius.html

	Example:
	t1=np.radians([-30,90,210])
	P=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1))])) #equilateral triangle
	CircumCenter=[0,0] #symmetry
	CircumR=CircumRadius(P)
	plt.plot(np.hstack([P[:,0],P[0,0]]),np.hstack([P[:,1],P[0,1]]),color=[0.5,0,0]) #plot triangle edges
	plt.scatter(P[:,0],P[:,1],color=[1,0,0]) #plot triangle points
	plt.scatter(CircumCenter[0],CircumCenter[1],color=[0,0,1]) #plot InCenter
	t2=np.linspace(0,2*np.pi,100) #for circle plotting
	plt.plot(CircumCenter[0]+CircumR*np.cos(t2),CircumCenter[1]+CircumR*np.sin(t2),ls='--',lw=2,color=[0,0,0]) #plot InCircle
	plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid
	'''
	A=P[0,:]; B=P[1,:]; C=P[2,:]
	a=np.linalg.norm(B-C); b=np.linalg.norm(A-C); c=np.linalg.norm(A-B);
	CircumR=a*b*c/np.sqrt((a+b+c)*(a+b-c)*(a+c-b)*(b+c-a))
	return CircumR
def OutFacingNormal(P,TriInCenter):
	'''
	Input:
	TriInCenter [x,y] of a triangle incenter
	P - 2x2 [[x1,y1], points on triangle of the same edge
			 [x2,y2]]

	Output:
	n - unit vector [x,y] normal to the edge specified by P, facing out of the triangle

	Example:
	t1=np.radians([-30,90,210])
	Ptri=np.transpose(np.vstack([(np.cos(t1)),(np.sin(t1))])) #equilateral triangle
	Pedge=Ptri[:2,:]
	InCenter,InR=InCenter(Ptri)
	q=np.mean(Pedge,axis=0) #mean by column
	n=OutFacingNormal(Pedge,InCenter)
	plt.plot(np.hstack([Ptri[:,0],Ptri[0,0]]),np.hstack([Ptri[:,1],Ptri[0,1]]),color=[0.5,0,0]) #plot triangle
	plt.scatter(Pedge[:,0],Pedge[:,1],color=[1,0,0]) #plot edge points
	plt.scatter(InCenter[0],InCenter[1],color=[0,0,1]) #plot InCenter
	plt.quiver(q[0],q[1],n[0],n[1])
	plt.grid(color='k',linestyle='-',linewidth=0.2) #add grid
	'''
	q=ProjPnt2Line(P,TriInCenter);
	n=(q-TriInCenter)/np.linalg.norm(q-TriInCenter);
	return n
def RotateVector(n,Theta):
	'''
	Input:
		n - row vector [x,y]
		Theta - angle in radians to rotate by
	Output:
		v - row rotated vector [x,y]
	'''
	Q=np.array([[np.cos(Theta),-np.sin(Theta)],
			   [np.sin(Theta),np.cos(Theta)]]) #Rotation matrix
	v=np.transpose(np.matmul(Q,np.transpose(n)))
	return v