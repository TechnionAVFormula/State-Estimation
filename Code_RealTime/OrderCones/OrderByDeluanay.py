import math
import numpy as np
from scipy.spatial import Delaunay
import copy

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

## classes and enums from our utilities:
from StateEst_Utils.config import CONFIG, IS_DEBUG_MODE 
from StateEst_Utils.ConeType import ConeType


# Get proper Enum:
YELLOW 		 = ConeType.YELLOW.value #messages.perception.Yellow
BLUE 		 = ConeType.BLUE.value #messages.perception.Blue
ORANGE 		 = ConeType.ORANGE_BIG.value #messages.perception.OrangeBig

FAKECONE = -10

Y_ASCII = 121
B_ASCII = 98

ITERATION_AMOUNT_FOR_FAILURE = 5

def orderByDeluanay(Cones, CarState, OrderByDeluanayParams):
    '''
    Input:
        Cones       - iterable group of cones, where each cone contains position.x, position.y and type
        CarState    - list of car properties containting CarState.x, CarState.y, and theta

    Output:
        returnBlue      - np array of ordered blue cones in same format as input
        returnYellow    - np array of ordered yellow cones in same format as input
        returnLostBlue  - np array of unordered blue cones in same format as input
        returnLostYellow- np array of unordered yellow cones in same format as input
        success - 1 if succesful delaunay, 0 if not
    '''

    # Parse Params:
    MaxIterations = OrderByDeluanayParams['MaxIterations'] # and so on...



    # Check input:
    if len(Cones) < 1:
        return [], [], [], [], 0

    #Convert Cones from input format to np array: 
    nCones = len(Cones)
    numCones = np.empty([len(Cones), 4])
    numCar = [CarState.position.x ,CarState.position.y]
    numVel = [math.cos(CarState.theta) ,math.sin(CarState.theta)]
    for i in range(nCones):
        # numCones[i][0] = Cones[i].position.x
        # numCones[i][1] = Cones[i].position.y
        numCones[i][0] = Cones[i].position[0]
        numCones[i][1] = Cones[i].position[1]
        numCones[i][3] = i
        if Cones[i].type == YELLOW:
            numCones[i][2] = Y_ASCII
        elif Cones[i].type == BLUE:
            numCones[i][2] = B_ASCII 

    #Initialize ordering algorithim
    mt = MapTrack(numCones, numCar, numVel)

    # Perform ordering:
    res = mt.OrderCones()
    
    # if result is -1 the algorithm failed:
    if res==-1:
        return [], [], [], [], 0

    #get output from algorithm
    bCones, yCones = mt.OrderedBlueCones, mt.OrderedYellowCones
    bLostCones, yLostCones = mt.LostBlue, mt.LostYellow

    #some cones get ordered several times, this makes sure we only take first ordering
    hasAppeard = np.zeros(nCones)

    #Create output for ordered yellow cones
    order = 0
    if yCones.shape[0] == 0:
        returnYellow = []
    else:
        returnYellow = np.empty([yCones.shape[0]],dtype=object)
        for cone in yCones:
            if cone[3] != FAKECONE and hasAppeard[int(cone[3])] != 1:
                hasAppeard[int(cone[3])] = 1
                returnYellow[order] = Cones[int(cone[3])]
                order += 1

    #Create output for ordered blue cones
    order = 0
    if bCones.shape[0] == 0:
        returnBlue = []
    else:
        returnBlue = np.empty([bCones.shape[0]], dtype=object)
        for cone in bCones:
            if cone[3] != FAKECONE and hasAppeard[int(cone[3])] != 1:
                hasAppeard[int(cone[3])] = 1
                returnBlue[order] = Cones[int(cone[3])]
                order += 1	
    
    #Create output for unordered blue cones
    blueOrder = 0
    returnLostBlue = np.empty([bLostCones.shape[0]], dtype=object)
    for cone in bLostCones:
        returnLostBlue[blueOrder] = Cones[int(cone[3])]
        blueOrder += 1
        
    #Create output for unordered yellow cones
    yellowOrder = 0
    returnLostYellow = np.empty([yLostCones.shape[0]], dtype=object)
    for cone in yLostCones:
        returnLostYellow[yellowOrder] = Cones[int(cone[3])]
        yellowOrder += 1

    #if a cone is ordered several times, the respective array will contain none. So we erase them here
    returnBlue = returnBlue[returnBlue[:]!=None]
    returnYellow = returnYellow[returnYellow[:]!=None]

    return returnBlue, returnYellow, returnLostBlue, returnLostYellow, 1


class MapTrack:
    def __init__(self,Cones,CarCG,CarVel,SphereR=10000, CarLength=10):
        '''C'tor of MapTrack Object, 
        
        Input:
        
         '''


        self.Cones=Cones #matrix mx3 [x,y,color]. color = (blue)/Y_ASCII(yellow)
        self.CarCG=CarCG #[x,y]
        self.CarDir=CarVel/np.linalg.norm(CarVel) #[x,y] of car direction


        #Geomtetric parameters
        self.SphereR=SphereR #10000 is an expiramental number
        self.CarLength=CarLength#10 is an expiramental number

        #initalize output of algorithm
        self.Cones4Calc=self.FindCones4Calc() #cones from which track is calculated.
        self.OrderedBlueCones=np.array([]) #Calculated in OrderCones
        self.OrderedYellowCones=np.array([]) #Calculated in OrderCones
        self.MidPoints=np.array([]) #Calculated in FindMidPoints

        self.LostBlue = np.array([])
        self.LostYellow = np.array([])
        
    def FindCones4Calc(self,Angle4FakeCones=np.pi/3):
        #Preprocessing
        Cones=SphereFilter(self.Cones,self.CarCG,self.SphereR)[0] #filtering for radius around car


        '''Not in use in this version.
        Previous use : Note which cones are behind the car:'''
        #Fix new points - fake cones - if no blue/yellow cones are found
        BlueCones=Cones[(Cones[:,2]==B_ASCII),:]
        FilteredBlueCones=BackFilter(BlueCones, self.CarCG, self.CarDir)
        FrontBlueCones = FilteredBlueCones[0]
        self.BackBlueCones = FilteredBlueCones[1]
        
        YellowCones = Cones[(Cones[:, 2] == Y_ASCII), :]
        FilteredYellowCones = BackFilter(YellowCones, self.CarCG, self.CarDir)
        FrontYellowCones = FilteredYellowCones[0]
        self.BackYellowCones = FilteredYellowCones[1]
        
        BaddFlag=FrontBlueCones.size==0
        YaddFlag=FrontYellowCones.size==0
        if BaddFlag:
            BNew=self.CarCG+self.CarLength*RotateVector(self.CarDir,+Angle4FakeCones) #add cone to the left
            Cones=np.vstack([Cones,np.hstack([BNew,B_ASCII,FAKECONE])]) #add fake yellow cone @ the buttom of matrix
        if YaddFlag: #no yellow cones exist
            YNew=self.CarCG+self.CarLength*RotateVector(self.CarDir,-Angle4FakeCones) #add cone to the right
            Cones=np.vstack([Cones,np.hstack([YNew,Y_ASCII,FAKECONE])]) #add fake yellow cone @ the buttom of matrix

        self.Cones4Calc=Cones
        return Cones
        
    def OrderCones(self,MaxItrAmnt=25,CostThreshold=1,ColorCostWeight=0.9, \
                    TriangleRadiiRatioThreshold=20):
        '''
        Input:
        self

        Output:
        to class:
        BCones - blue cones in order for interpolation [x,y]
        Ycones - yellow cones in order for interpolation [x,y]
        '''
        if self.Cones4Calc.shape[0] < 3:
            return -1
            
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
        if ID == -1: return -1  # no cones in sight. couldnt build a track

        graph = CreateGraph(DT, Cones[:,2])

        #Important note: DT.find_simplex returns -1 if point is outside triangulation.
        #But we use setdiff1d in both TriOne and FindNextTriangle so NewID can be of empty.
        NewID,Newu,NewCrossEdge=TriOne(DT,Cones[:,2],ID,self.CarDir,graph,ColorCostWeight)
        if NewID.size==0: return -1 #crossed into no-man's land

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
            NewID,Newu,NewCrossEdge,Cost,TriangleRadiiRatio=FindNextTriangle(DT,Cones[:,2],NewID,Newu,NewCrossEdge,graph,ColorCostWeight)
            if Cost > CostThreshold or TriangleRadiiRatio > TriangleRadiiRatioThreshold or NewID.size==0:
                if Itr < ITERATION_AMOUNT_FOR_FAILURE:
                    return -1 #Failed
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

        #delete fake cones
        if YCones.shape[0] != 0:
            YCones = YCones[(YCones[:, 3] != FAKECONE), :]
        if BCones.shape[0] != 0:
            BCones = BCones[(BCones[:, 3] != FAKECONE), :]

        #Output to class
        self.OrderedBlueCones=BCones
        self.OrderedYellowCones=YCones
        
        #Output Filtered Cones to class
        self.SetFilteredCones()

    def SetFilteredCones(self):
        '''
        Input:
        self

        Output:
        to class:
        LostBlue - blue cones in order for interpolation [x,y]
        LostYellow - yellow cones in order for interpolation [x,y]

        This function is the function to take care of unordered cones and the reasons
        why they were not chosen
        '''
        isFiltered = np.zeros(self.Cones.shape[0])
        nBlueCones = self.Cones[(self.Cones[:,2]==B_ASCII),:].shape[0]
        nYellowCones = self.Cones[(self.Cones[:,2]==Y_ASCII),:].shape[0]
        nFilteredBlueCones = 0
        nFilteredYellowCones = 0

        #flag all ordered cones, count every cone that is ordered, exactly once,
        #because it can be ordered several times    
        for cone in self.OrderedBlueCones:
            if cone[3] != FAKECONE and isFiltered[int(cone[3])] == 0:
                isFiltered[int(cone[3])] = 1
                nFilteredBlueCones += 1
        for cone in self.OrderedYellowCones:
             if cone[3] != FAKECONE and isFiltered[int(cone[3])] == 0:
                isFiltered[int(cone[3])] = 1
                nFilteredYellowCones += 1
                    
                
        #create array for lost cones from cones not flaged
        self.LostBlue = np.zeros([nBlueCones - nFilteredBlueCones, 4])
        self.LostYellow = np.zeros([nYellowCones - nFilteredYellowCones, 4])
        bCounter = 0
        yCounter = 0
        for i in range(0, isFiltered.size):
            if int(isFiltered[i]) == 0:
                if self.Cones[i][2] == B_ASCII:
                    self.LostBlue[bCounter] = self.Cones[i]
                    bCounter += 1
                elif self.Cones[i][2] == Y_ASCII:
                    self.LostYellow[yCounter] = self.Cones[i]
                    yCounter += 1
        
    #Associated functions with class ConesDT
def TriOne(DT,ConesColors,ID,CarDir,graph,ColorCostWeight):
    '''
    Input:
    DT - DelaunayTriangulation containing DT.points and DT.simplices
    ConesColors - colors of cones (B_ASCII for blue, Y_ASCII for yellow) with same
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
    J1 += TraverseGraph(V[EdgesInd[0]], graph, 5, 5)
    J2=EdgeCost(IC,P[EdgesInd[1,:],:],CarDir,ColorCostWeight)
    J2 += TraverseGraph(V[EdgesInd[1]], graph, 5, 5)
    J3=EdgeCost(IC,P[EdgesInd[2,:],:],CarDir,ColorCostWeight)
    J3 += TraverseGraph(V[EdgesInd[2]], graph, 5, 5)

    J=np.hstack([J1,J2,J3]); JminInd=np.argmin(J)
    NewCrossEdge=V[EdgesInd[JminInd,:]] #obtain Edge Vertices to cross (point indcies [V1,V2])
    Newu=OutFacingNormal(P[EdgesInd[JminInd,:],:2],IC) #calculate normal to cross edge
    Attachments=np.where(np.any(DT.simplices==NewCrossEdge[0],axis=1) &\
                            np.any(DT.simplices==NewCrossEdge[1],axis=1)) #find IDs that are connected to edge
    NewID=np.squeeze(np.setdiff1d(Attachments,ID)) #NewID - New Triangle
    return NewID,Newu,NewCrossEdge

def FindNextTriangle(DT,ConesColors,ID,Dir,CrossEdge,graph,ColorCostWeight):
    '''
    Input:
    DT - DelaunayTriangulation containing DT.points and DT.simplices
    ConesColors - colors of cones (B_ASCII/98 for blue, Y_ASCII/121 for yellow) with same
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
    TriangleRadiiRatio - ratio between Circumcenter/InCenter radii
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

    #Start Backtracking Algo into further triangles:
    J1=EdgeCost(IC,P[EdgesInd[0,:],:],Dir,ColorCostWeight)
    J1 += TraverseGraph(V[EdgesInd[0]], graph, 5, 5)
    J2=EdgeCost(IC,P[EdgesInd[1,:],:],Dir,ColorCostWeight)
    J2 += TraverseGraph(V[EdgesInd[1]], graph, 5, 5)

    J=np.hstack([J1,J2]); MinCost=np.min(J); JminInd=np.argmin(J) #find Edge to cross by row index in Edges
    NewCrossEdge=V[EdgesInd[JminInd,:]] #obtain Edge Vertices to cross (point indcies [V1,V2])
    NewDir=OutFacingNormal(P[EdgesInd[JminInd,:],:2],IC) #calculate normal to cross edge
    Attachments=np.where(np.any(DT.simplices==NewCrossEdge[0],axis=1) &\
                            np.any(DT.simplices==NewCrossEdge[1],axis=1)) #find IDs that are connected to edge
    NewID=np.squeeze(np.setdiff1d(Attachments,ID)) #NewID - New Triangle

    if NewID.size==0:
        TriangleRadiiRatio=0 #filler
        return NewID,NewDir,NewCrossEdge,MinCost,TriangleRadiiRatio

    NewV=DT.simplices[NewID,:] #find vertex indcies of NewID (=NewTriangle)
    #    NewV=NewV.reshape(3)
    NewP=DT.points[NewV,:] #find points correlating to new vertex indcies
    
    '''Note to self: check how time consuming is this computation:'''
    TriangleRadiiRatio=CircumRadius(NewP)/InCenter(NewP)[1] #calculate TriangleRadiiRatio of new triangle
    return NewID,NewDir,NewCrossEdge,MinCost,TriangleRadiiRatio


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
    ConesColors=np.array([B_ASCII,Y_ASCII,B_ASCII])
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
    elif leftPoint[2] == Y_ASCII:
        J=J+1
    else: #not the same color
        J=J-1.5
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
    Cones=np.array([[-1,0.5,B_ASCII],
                    [1,1,Y_ASCII],
                    [1,-0.6,B_ASCII]])
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
    a=np.linalg.norm(B-C); b=np.linalg.norm(A-C); c=np.linalg.norm(A-B)
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
    q=ProjPnt2Line(P,TriInCenter)
    n=(q-TriInCenter)/np.linalg.norm(q-TriInCenter)
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

class Point:
    def __init__(self, order, x, y, color):
        self.order = order
        self.x = x
        self.y = y
        self.color = color

class Node:
    edge1=[0,0]
    edge2=[0,0]
    cost1=0
    cost2=0

def UpdateGraphByEdge(p1,p2,p3,graph):
    '''
    Update node in graph defined by the edge in the triangle

    Input:
    p1,p2-Two points of the edge that define the node
    p3-third point in triangle
    graph

    Output:
    in graph the respective node is created
    '''

    #understand which point is the left one in the edge by doing cross product
    #between:
    #u which is the vector from middle of edge to middle of triangle
    #dir1 which is vector from middle of edge and p1
    middle = np.array([(p1.x+p2.x+p3.x)/3,(p1.y+p2.y+p3.y)/3])
    center1 = np.array([(p1.x+p2.x)/2,(p1.y+p2.y)/2])
    u = middle - center1
    dir1 = np.array([center1[0]-p1.x,center1[1]-p1.y])
    dir1 = dir1 / math.sqrt(dir1[0]*dir1[0]+dir1[1]*dir1[1])
    u = u / math.sqrt(u[0]*u[0]+u[1]*u[1])

    node = Node()
    Newu=-1*OutFacingNormal(np.array([[p1.x,p1.y],[p2.x,p2.y]]),middle)

    #if p1 is left point in the edge
    if np.cross(u,dir1) > 0:
        node.edge1 = [p1.order, p3.order]
        node.edge2 = [p3.order, p2.order]

        edgeCones1 = np.array([[p1.x,p1.y,p1.color],[p3.x,p3.y,p3.color]])
        node.cost1 = EdgeCost(middle, edgeCones1, Newu, 0.5 )

        edgeCones2 = np.array([[p2.x,p2.y,p2.color],[p3.x,p3.y,p3.color]])
        node.cost2 = EdgeCost(middle, edgeCones2, Newu, 0.5 )

        graph[p1.order, p2.order] = node

    #if p1 is right point in the edge
    else:
        node.edge1 = [p2.order, p3.order]
        node.edge2 = [p3.order, p1.order]

        edgeCones1 = np.array([[p2.x,p2.y,p2.color],[p3.x,p3.y,p3.color]])
        node.cost1 = EdgeCost(middle, edgeCones1, Newu, 0.5 )

        edgeCones2 = np.array([[p1.x,p1.y,p1.color],[p3.x,p3.y,p3.color]])
        node.cost2 = EdgeCost(middle, edgeCones2, Newu, 0.5 )

        graph[p2.order, p1.order] = node


def CreateGraph(DT, ConeColor):
    '''
    Create graph from Delaunay Triangulation

    Input:
    DT-Delaunay Triangulation of cones
    ConeColor-array containing color of cones

    Output:
    graph-two dimensional array containg nodes with pointers and costs, each id of a
    node consists of two numbers, left cone id and right cone id
    '''

    graph = np.empty([DT.points.shape[0],DT.points.shape[0]], dtype=object)

    #go through each triangle and every edge of that triangle is a node
    for tria in DT.simplices:

        triangle = DT.points[tria]

        p1 = Point(tria[0],triangle[0][0],triangle[0][1],ConeColor[tria[0]])
        p2 = Point(tria[1],triangle[1][0],triangle[1][1],ConeColor[tria[1]])
        p3 = Point(tria[2],triangle[2][0],triangle[2][1],ConeColor[tria[2]])

        #update the graph
        UpdateGraphByEdge(p1,p2,p3,graph)
        UpdateGraphByEdge(p3,p2,p1,graph)
        UpdateGraphByEdge(p1,p3,p2,graph)

    return graph
        
def TraverseGraph(edge, graph, depth, maxDepth):
    '''
    Recursive function

    Input:
    edge-edge from which car entered the respective triangle containing it
    graph-the graph
    depth-depth of call
    maxDepth-max depth possible, used for calculating cost

    Output:
    cost-cost of minimum path continuing the edge
    '''
    #end of depth
    if depth == 0:
        return 5
    node = graph[edge[0],edge[1]]

    #if node is fake
    if node == None:
        return 5

    #get the recursive cost of both edges
    cost1 = node.cost1 + (depth/maxDepth)*TraverseGraph(node.edge1, graph, depth - 1, maxDepth)
    cost2 = node.cost2 + (depth/maxDepth)*TraverseGraph(node.edge2, graph, depth - 1, maxDepth)

    #return minimum
    if cost1 < cost2:
        return cost1
    else:
        return cost2

if __name__ == "__main__":
	import tkinter as tk
	from timeit import default_timer as timer

	TESTBLUE = 0
	TESTYELLOW = 1
	TESTCAR = 2

	class Position:
			x = 0
			y = 0

	class TestCarState:
		position = Position()
		theta = 0

	class TestCone:
		position = object
		type = 0

	def printMap(bluePoints , yellowPoints , blueLost ,yellowLost, x, y, Vx, Vy):
		root = tk.Tk()

		my_canvas = tk.Canvas(root, width=1200, height=600)

		order = 1
		for cone in bluePoints:
			my_canvas.create_oval(cone.position.x-3,cone.position.y-3,cone.position.x+3,cone.position.y+3,fill="blue")
			label = tk.Label(master = root,text=str(order))
			order += 1
			label.place(x=cone.position.x-3 - 5,y= cone.position.y - 30)
		for cone in blueLost:
			my_canvas.create_oval(cone.position.x-3,cone.position.y-3,cone.position.x+3,cone.position.y+3,fill="blue")
			label = tk.Label(master = root,text=str(0))
			label.place(x=cone.position.x - 5,y= cone.position.y - 30)
		order = 1
		for cone in yellowPoints:
			my_canvas.create_oval(cone.position.x-3,cone.position.y-3,cone.position.x+3,cone.position.y+3,fill="yellow")
			label = tk.Label(master = root,text=str(order))
			order += 1
			label.place(x=cone.position.x - 5,y= cone.position.y - 30)
		for cone in yellowLost:
			my_canvas.create_oval(cone.position.x-3,cone.position.y-3,cone.position.x+3,cone.position.y+3,fill="yellow")
			label = tk.Label(master = root,text=str(0))
			label.place(x=cone.position.x - 5,y= cone.position.y - 30)


		my_canvas.create_oval(x-3,y-3,x+3,y+3,fill="purple")
		my_canvas.create_line(x,y,x+Vx,y+Vy,fill="red")
		my_canvas.pack()

		root.mainloop()

	def printPreMap(Cones, x, y, Vx, Vy):
		root = tk.Tk()

		my_canvas = tk.Canvas(root, width=1200, height=600)

		for cone in Cones:
			if cone.type == BLUE:
				my_canvas.create_oval(cone.position.x-3,cone.position.y-3,cone.position.x+3,cone.position.y+3,fill="blue")
			if cone.type == YELLOW:
				my_canvas.create_oval(cone.position.x-3,cone.position.y-3,cone.position.x+3,cone.position.y+3,fill="yellow")


		my_canvas.create_oval(x-3,y-3,x+3,y+3,fill="purple")
		my_canvas.create_line(x,y,x+Vx,y+Vy,fill="red")
		my_canvas.pack()

		root.mainloop()

	def RunTest(testName):
		toRead=open(testName, 'r')
		pointStr = toRead.readline()
		a = pointStr.split(" ")
		x = int(a[0])
		y = int(a[1])
		pointStr = toRead.readline()
		a = pointStr.split(" ")
		Vx = int(a[0])
		Vy = int(a[1])

		cones = []
		pointStr = toRead.readline()
		while pointStr != '':
			a = pointStr.split(" ")
			newCone = TestCone()
			newCone.position = Position()
			newCone.position.x = int(a[1])
			newCone.position.y = int(a[2])
			if int(a[3]) == TESTBLUE:
				newCone.type = BLUE
			elif int(a[3]) == TESTYELLOW:
				newCone.type = YELLOW
			cones.append(newCone)
			pointStr = toRead.readline()

		toRead.close()
		carState = TestCarState()
		carState.position.x = x
		carState.position.y = y
		if Vx > 0:
			carState.theta = math.atan(Vy/Vx)
		elif Vx < 0:
			carState.theta = math.atan(Vy/Vx) + math.pi
		else:
			carState.theta = math.pi/2

		beg = timer()
		bluePoints, yellowPoints, LostBlue, LostYellow, success = orderByDeluanay( cones,carState)
		print(timer()-beg)
		printPreMap( cones, x, y, Vx, Vy)
		printMap(bluePoints , yellowPoints , LostBlue ,LostYellow, x, y, Vx, Vy)
    
    #run single test
    #RunTest("Test13.txt")

    #run all tests
	for i in range(1, 14):
		RunTest("Test" + str(i) + ".txt")
