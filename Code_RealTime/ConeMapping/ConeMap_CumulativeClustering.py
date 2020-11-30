from sklearn.cluster import OPTICS, cluster_optics_dbscan
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
from StateEst_Utils.ConeType import ConeType as ConeTypeEnum
4# messages is used to create a formula cone object:
from StateEst_Utils.MessagesClass import messages
# Copy operations so that get_real_cones() won't break when adding elements to list the pythonic way:
import copy

# Get proper Enum:
YELLOW 		 = ConeTypeEnum.YELLOW.value #messages.perception.Yellow
BLUE 		 = ConeTypeEnum.BLUE.value #messages.perception.Blue
ORANGE_BIG   = ConeTypeEnum.ORANGE_BIG.value #messages.perception.OrangeBig
ORANGE_SMALL = ConeTypeEnum.ORANGE_SMALL.value
UNKNOWN      = ConeTypeEnum.UNKNOWN.value

# ConesTypes = ["YELLOW","BLUE","ORANGE_BIG","ORANGE_SMALL"]
ConesTypes = [ YELLOW, BLUE , ORANGE_BIG , ORANGE_SMALL ]



class ConeMap_CumulativeClustering(ConeMap_Base):
    def __init__(self):
        # Hypre Params:
        self.filterIterationWaitInterval = 10
        self.realConeThreshold = 4 # We better find a connection between those two
        self.SuperClusterRadius = 1


        # dynamic properties:
        # ==================
        self._cone_samples   = np.array( [] ) #accumaltive samples before clusstering
        self._super_clusters = dict([ (coneType, np.array( [] , dtype=object ) ) for coneType in ConesTypes ]) 
        self._call_counter = 0


    '''Sampling new cones'''
    def insert_new_points(  self  , cone_array   ):        
        self._call_counter = self._call_counter + 1
        self._cone_samples = np.append( self._cone_samples , cone_array )
        # Not every insert causes filttering.
        # we need to get a certein amount of samples - denoted by self.filterIterationWaitInterval:
        if ( self._call_counter >= self.filterIterationWaitInterval ):
            #set counter:
            self._call_counter = 0 
            #prepare cone samples for clustering module:
            #yellow_cones , blue_cones , orange_big_cones, orange_small_cones  = self.__prepare_cones4clusttering()
            PreparedConesByType = self.__prepare_cones4clusttering()
            # Clear old samples:
            self._cone_samples = np.array( [] )

            for coneType in ConesTypes:
                #Run clustering module:
                clusterObject  = ClusteringOptics( PreparedConesByType[coneType] ,self.realConeThreshold)
                #create SuperCluster objects from the clusters that we've found:
                newSuperClusters = optics2SuperClusters( clusterObject , PreparedConesByType[coneType] , coneType )   
                #Find and check if any of those new clusters conflicts with our old saved clusters. save the results after:
                self._super_clusters[coneType] = self._combine_superClusters( newSuperClusters , self._super_clusters[coneType] )

        return


    def _combine_superClusters(self , new_clusters , old_clusters):
        """_combine_superClusters From new and old cluster lists, decide which are the same, and kill any
        unnecessary data.


        Args:
            new_clusters (list of SuperCluster objects): just arrived.
            old_clusters (list of SuperCluster objects): were here before.

        Returns:
            list of SuperCluster objects: combined_clusters
        """

        combined_clusters = []

        for new_cluster in new_clusters:
            x_new = new_cluster.getX()
            y_new = new_cluster.getY()
            is_foundSimilarOldCluster = False
            similarOldCluster = None

            # Search for a similar super cluster from the old ones
            for old_cluster in old_clusters:
                x_old = old_cluster.getX() 
                y_old = old_cluster.getY()
                distance_sqrd = (x_old-x_new)**2 + (y_old-y_new)**2 

                # check if that's a similar old cluster:
                if (distance_sqrd <= self.SuperClusterRadius**2):
                    is_foundSimilarOldCluster = True
                    similarOldCluster = old_cluster
                    break

            if is_foundSimilarOldCluster:
                similarOldCluster.combine( new_cluster )
                combined_clusters.append( similarOldCluster )
            else:
                combined_clusters.append( new_cluster )
            
            # now check next new_cluster

        return combined_clusters


    # need to add the logic for "killing" old super cluster                     

    def __prepare_cones4clusttering(self):
        ColoredCones = dict([ (coneType , np.array([])) for coneType in ConesTypes ])

        for cone in self._cone_samples:
            element = [cone.position.x , cone.position.y]
            coneType = cone.type

            """Act regarding special types"""
            if cone.type in ConesTypes:
                pass
            elif cone.type == UNKNOWN:
                print(f"StateEstimation:ConeMap_CumulativeClustering: Got an unkown cone type")
                continue
            else:
                raise NameError(f"Got cone.type ={cone.type}. Not an expected cone type.")

            """The first time you add to the list, you can't use vstack"""
            if len(ColoredCones[coneType])==0:
                ColoredCones[coneType] = np.append(  ColoredCones[coneType] , element  )
            else:
                ColoredCones[coneType] = np.vstack(( ColoredCones[coneType] , element ))
        
        return ColoredCones


        yellow_cones = np.array([])
        blue_cones   = np.array([])
        orange_big_cones = np.array([])
        orange_small_cones = np.array([])
        for cone in self._cone_samples:
            element = [cone.position.x , cone.position.y]

            if cone.type == YELLOW :
                if len(yellow_cones) == 0:
                   yellow_cones=np.append(yellow_cones, element)
                else:
                    yellow_cones=np.vstack((yellow_cones, element))

            elif cone.type == BLUE :
                if len(blue_cones) == 0:
                   blue_cones=np.append(blue_cones, element)
                else:
                    blue_cones=np.vstack((blue_cones, element))


            elif cone.type == ORANGE_BIG:
                if len(orange_cones) == 0:
                   orange_cones=np.append(orange_cones, element)

                else:
                    orange_big_cones=np.vstack((orange_big_cones, element))
            
            elif cone.type==ORANGE_SMALL:
                if len(orange_small_cones) == 0:
                   orange_small_cones=np.append(orange_small_cones, element)
                else:
                    orange_small_cones=np.vstack((orange_small_cones, element))
            
            elif cone.type == UNKNOWN:
                print(f"StateEstimation:ConeMap_CumulativeClustering: Got an unkown cone type")

            else:
                raise Exception("StateEstimation:ConeMap_CumulativeClustering: Got cone with no type")
                
            '''!!! Add ORANGE_SMALL !!!'''


        return yellow_cones, blue_cones, orange_big_cones, orange_small_cones

    
    ## Also in base-class
    def get_all_samples( self ):
        return self._cone_samples

    def get_real_cones( self ): 
        coneArray = np.array([])
        for coneType in ConesTypes:
            for superCluster in self._super_clusters[coneType]:
                coneDict  = SuperCluster2Dict( superCluster )
                tempArray = np.array( [coneDict] )
                coneArray = np.append( coneArray , tempArray )                
        return coneArray
            

def SuperCluster2Dict(superCluster):
    '''Get information from cluster'''
    x       = superCluster.getX()   
    y       = superCluster.getY()  
    x_dev   = superCluster.getXDeviation()
    y_dev   = superCluster.getYDeviation()
    Type    = superCluster.getType()
    coneId  = superCluster.getId()
    '''passs information to Dict:'''
    tempStateCone = dict()
    tempStateCone["position"]           = (x,y)
    tempStateCone["position_deviation"] = (x_dev,y_dev)
    tempStateCone["cone_id"]            = coneId
    tempStateCone["type"]               = Type
    return tempStateCone

def SuperCluster2StateCone(superCluster):
    '''Get information from cluster'''
    x       = superCluster.getX()   
    y       = superCluster.getY()  
    x_dev   = superCluster.getXDeviation()
    y_dev   = superCluster.getYDeviation()
    Type    = superCluster.getType()
    coneId  = superCluster.getId()
    '''passs information to cone:'''
    tempStateCone = messages.state_est.StateCone() 
    tempStateCone.position.x = x
    tempStateCone.position.y = y
    tempStateCone.position_deviation = max( x_dev , y_dev )
    tempStateCone.cone_id    = coneId
    tempStateCone.type       = Type
    return tempStateCone

def optics2SuperClusters(clust, cones, coneType):
    superClusters_LIST = []
    if clust==None:
        return superClusters_LIST 

    maxLabel = clust.labels_.max() 
    minLabel = clust.labels_.min() 
    for k in np.arange(minLabel , maxLabel+1): #Go over all classes k:
        #Get info from clustering object:
        indices_k = clust.labels_==k # array of all indices where vludter index is k
        cones_k = cones[indices_k] # all cones with cluster index k
        x_vec  = cones_k[:,0]
        y_vec  = cones_k[:,1]
        #Compute Clusters params:
        mean_x = np.mean(x_vec)
        mean_y = np.mean(y_vec)
        standart_deviation_x = np.std( x_vec )
        standart_deviation_y = np.std( y_vec )
        weight = cones_k.shape[0] # number of cones_k
        #Create SuperCluster with those params::
        newSuperCluster = SuperCluster( coneType , weight , mean_x,mean_y , standart_deviation_x,standart_deviation_y)   
        superClusters_LIST.append(newSuperCluster)
    return superClusters_LIST

def debug_print_all_list(givenList):
    print("---")
    for element in givenList:        
        if element == None:
            pass
        else:            
            try:

                if isinstance(element,dict):
                    val = element["position"]
                else:
                    val = element.position
                print(f"{val}")    
            
            except:
                print("passed")
# missing main