import numpy as np
import math

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

# Clusttering method
try:
    from Code_RealTime.ConeMapping.ClusteringOptics import ClusteringOptics
except:
    from ClusteringOptics import ClusteringOptics

# base class:
try:
    from Code_RealTime.ConeMapping.ConeMap_Base import ConeMap_Base
except:
    from ConeMap_Base import ConeMap_Base

# SuperCluster object for keeping track on previous detections:
try:
    from Code_RealTime.ConeMapping.SuperCluster import SuperCluster
except:    
    from SuperCluster import SuperCluster


## classes and enums from our utilities:
from StateEst_Utils.config import CONFIG, IS_DEBUG_MODE 
from StateEst_Utils.ConeType import ConeType
# messages is used to create a formula cone object:
from StateEst_Utils.MessagesClass import messages


# Get proper Enum:
YELLOW 		 = ConeType.YELLOW.value #messages.perception.Yellow
BLUE 		 = ConeType.BLUE.value #messages.perception.Blue
ORANGE_BIG   = ConeType.ORANGE_BIG.value #messages.perception.OrangeBig
ORANGE_SMALL = ConeType.ORANGE_SMALL.value
UNKNOWN      = ConeType.UNKNOWN.value




class ConeMap_CumulativeClustering(ConeMap_Base):
    def __init__(self):
        # Hypre Params:
        self.filterFrequency = 10
        self.realConeThreshold = 4
        self.SuperClusterRadius = 1


        # dynamic propertise:
        self._cone_samples = np.array( [] ) #accumaltive ssamples before clusstering

        self._blue_super_cluster         = np.array( [] )
        self._orange_big_super_cluster   = np.array( [] )
        self._orange_small_super_cluster = np.array( [] )
        self._yellow_super_cluster       = np.array( [] )

        self._call_counter = 0


    '''Sampling new cones'''
    def insert_new_points(  self  , cone_array   ):
        
        self._call_counter = self._call_counter + 1
        self._cone_samples = np.append( self._cone_samples , cone_array )


        # Not every insert causes filttering.
        # we need to get a certein amount of samples - denoted by self.filterFrequency:
        if ( self._call_counter >=  self.filterFrequency  ):

            yellow_cones , blue_cones , orange_big_cones, orange_small_cones  = self.__prepare_cones4clusttering()
            blue_clusterList            = ClusteringOptics(blue_cones  ,self.realConeThreshold)
            yellow_clusterList          = ClusteringOptics(yellow_cones,self.realConeThreshold)
            orange_big_clusterList      = ClusteringOptics(orange_big_cones,self.realConeThreshold)
            orange_small_clusterList    = ClusteringOptics(orange_small_cones,self.realConeThreshold)

            blue_super_clusters        = _optics2SuperClusters(blue_clusterList,  BLUE)
            yellow_super_clusters      = _optics2SuperClusters(yellow_clusterList ,  YELLOW)
            orange_big_super_clusters  = _optics2SuperClusters(orange_big_clusterList ,  ORANGE_BIG)
            orange_small_super_clusters= _optics2SuperClusters(orange_small_clusterList ,  ORANGE_SMALL)

            self._blue_super_cluster            = self._combine_superClusters(blue_super_clusters         , self._blue_super_cluster)
            self._yellow_super_cluster          = self._combine_superClusters(yellow_super_clusters       , self._yellow_super_cluster)
            self._orange_big_super_cluster      = self._combine_superClusters(orange_big_super_clusters   , self._orange_big_super_cluster)
            self._orange_small_super_cluster    = self._combine_superClusters(orange_small_super_clusters , self._orange_small_super_cluster)
        
            #do svm here

    def _combine_superClusters(self , new_clusters , old_clusters):
        for old_cluster in old_clusters:
            x_old = old_cluster.getX() 
            y_old = old_cluster.getY()
            for new_cluster in new_clusters:
                x_new = new_cluster.getX()
                y_new = new_cluster.gety()
                distance_sqrd = (x_old-x_new)**2 + (y_old-y_new)**2 
                if (distance_sqrd < self.SuperClusterRadius**2):
                    old_cluster.combine(new_cluster) 
    # need to add the logic for "killing" old super cluster                     

    def __prepare_cones4clusttering(self):
        yellow_cones = np.array([])
        blue_cones   = np.array([])
        orange_big_cones = np.array([])
        orange_small_cones = np.array([])
        for cone in self._cone_samples:
            element = [cone.position.x , cone.position.y]

            if cone.type==YELLOW :
                if len(yellow_cones) == 0:
                   yellow_cones=np.append(yellow_cones, element)
                else:
                    yellow_cones=np.vstack((yellow_cones, element))

            if cone.type==BLUE :
                if len(blue_cones) == 0:
                   blue_cones=np.append(blue_cones, element)
                else:
                    blue_cones=np.vstack((blue_cones, element))

            if cone.type==ORANGE_BIG:
                if len(orange_big_cones) == 0:
                   orange_big_cones=np.append(orange_big_cones, element)
                else:
                    orange_big_cones=np.vstack((orange_big_cones, element))

            if cone.type==ORANGE_SMALL:
                if len(orange_small_cones) == 0:
                   orange_small_cones=np.append(orange_small_cones, element)
                else:
                    orange_small_cones=np.vstack((orange_small_cones, element))

        return yellow_cones, blue_cones, orange_big_cones, orange_small_cones

    
    ## Also in base-class
    def get_all_samples( self ):
        return self._cone_samples

    def get_real_cones( self ): 
        cone_array = np.array([]) #contains all types
        '''Shani : Do it for 4 colors:'''
        for superCluster in self._blue_super_cluster:
            '''Get information from cluster'''
            x    = superCluster.x   
            y    = superCluster.y  
            Type = superCluster.Type
            '''passs information to cone:'''
            cone = messages.state_est.StateCone
            cone.position[0] = x
            cone.position[1] = y
            cone.type = Type
            '''add to array'''
            cone_array = np.append(cone_array , cone)
        return cone_array

def _optics2SuperClusters(cluster_list, Type):
    superClusters_list = []
    if cluster_list==None:
        return superClusters_list 
    for cluster in cluster_list:
        mean_x = np.mean(cluster[:,0])
        mean_y = np.mean(cluster[:,1])
        weight = np.shape(cluster)[0]
        newSuperCluster = SuperCluster(Type , weight , mean_x , mean_y)   
        superClusters_list.append(newSuperCluster)
    return superClusters_list

# missing main