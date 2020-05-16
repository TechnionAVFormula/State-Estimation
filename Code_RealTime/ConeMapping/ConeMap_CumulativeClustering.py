import numpy as np
import math

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

# Clusttering method
from ClusteringOptics import ClusteringOptics
# base class:
from ConeMap_Base import ConeMap_Base
# SuperCluster object for keeping track on previous detections:
from SuperCluster import SuperCluster


## classes and enums from our utilities:
from StateEst_Utils.config import CONFIG, IS_DEBUG_MODE 
from StateEst_Utils.ConeType import ConeType
# messages is used to create a formula cone object:
from StateEst_Utils.MessagesClass import messages


# Get proper Enum:
YELLOW 		 = ConeType.YELLOW #messages.perception.Yellow
BLUE 		 = ConeType.BLUE #messages.perception.Blue
ORANGE_BIG   = ConeType.ORANGE_BIG #messages.perception.OrangeBig
ORANGE_SMALL = ConeType.ORANGE_SMALL 
UNKNOWN      = ConeType.UNKNOWN



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
            blue_clust          = ClusteringOptics(blue_cones  ,self.realConeThreshold)
            yellow_clust        = ClusteringOptics(yellow_cones,self.realConeThreshold)
            orange_big_clust    = ClusteringOptics(orange_big_cones,self.realConeThreshold)
            orange_small_clust  = ClusteringOptics(orange_small_cones,self.realConeThreshold)

            self._blue_super_cluster = self._optics2SuperCluster(blue_clust ,  BLUE)
             # same for all other colors
            '''Shani gets: blue_clust        
                            yellow_clust      
                            orange_big_clust  
                            orange_small_clust 
                        For all of the apply convertion '''
  
                


            #do svm here

    def __prepare_cones4clusttering(self):
        yellow_cones = np.array([])
        blue_cones   = np.array([])
        orange_big_cones = np.array([])
        orange_small_cones = np.array([])
        for cone in self._cone_samples:
            element = [cone.x , cone.y]

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

    def _optics2SuperClusters(opticsClusters, Type):
        super_clusters_array = np.array([])
        '''Shani: 
        Get an optic cluster object an get from it:
        x , y , weight
        and pass it to a new supercluster'''
        for cluster in opticsClusters:
            newSuperCluster = SuperCluster(Type , Weight_sum , x_mean , y_mean)   
            super_clusters_array = np.append(super_clusters_array , newSuperCluster) 
        return super_clusters_array


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

''' Shani: definition for SuperCluster2Cone '''