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
from StateEst_Utils.ConeType import ConeType
# messages is used to create a formula cone object:
from StateEst_Utils.MessagesClass import messages
# Copy operations so that get_real_cones() won't break when adding elements to list the pythonic way:
import copy

# Get proper Enum:
YELLOW 		 = ConeType.YELLOW.value #messages.perception.Yellow
BLUE 		 = ConeType.BLUE.value #messages.perception.Blue
ORANGE_BIG   = ConeType.ORANGE_BIG.value #messages.perception.OrangeBig
ORANGE_SMALL = ConeType.ORANGE_SMALL.value
UNKNOWN      = ConeType.UNKNOWN.value

ConesTypes = ["YELLOW","BLUE","ORANGE_BIG","ORANGE_SMALL"]


class ConeMap_CumulativeClustering(ConeMap_Base):
    def __init__(self):
        # Hypre Params:
        self.filterFrequency = 10
        self.realConeThreshold = 4 # We better find a connection between those two
        self.SuperClusterRadius = 1


        # dynamic propertise:
        self._cone_samples = np.array( [] ) #accumaltive ssamples before clusstering

        self._blue_super_clusters         = np.array( [] )
        self._yellow_super_clusters       = np.array( [] )
        self._orange_big_super_clusters   = np.array( [] )
        self._orange_small_super_clusters = np.array( [] )
        

        self._call_counter = 0


    '''Sampling new cones'''
    def insert_new_points(  self  , cone_array   ):
        
        self._call_counter = self._call_counter + 1
        self._cone_samples = np.append( self._cone_samples , cone_array )


        # Not every insert causes filttering.

        # we need to get a certein amount of samples - denoted by self.filterFrequency:
        if ( self._call_counter >=  self.filterFrequency  ):
            #set counter:
            self._call_counter = 0 
            #prepare cone samples for clustering module:
            yellow_cones , blue_cones , orange_big_cones, orange_small_cones  = self.__prepare_cones4clusttering()
            # Clear old samples:
            self._cone_samples = np.array( [] )
            #Run clustering module:
            blue_clusterObject            = ClusteringOptics(blue_cones           ,self.realConeThreshold)
            yellow_clusterObject          = ClusteringOptics(yellow_cones         ,self.realConeThreshold)
            orange_big_clusterObject      = ClusteringOptics(orange_big_cones     ,self.realConeThreshold)
            orange_small_clusterObject    = ClusteringOptics(orange_small_cones   ,self.realConeThreshold)
            #create SuperCluster objects from the clusters that we've found:
            new_blue_super_clusters        = optics2SuperClusters(blue_clusterObject         ,blue_cones,          BLUE         )
            new_yellow_super_clusters      = optics2SuperClusters(yellow_clusterObject       ,yellow_cones,        YELLOW       )
            new_orange_big_super_clusters  = optics2SuperClusters(orange_big_clusterObject   ,orange_big_cones,    ORANGE_BIG   )
            new_orange_small_super_clusters= optics2SuperClusters(orange_small_clusterObject ,orange_small_cones,  ORANGE_SMALL )
            #Find and check if any of those new clusters conflicts with our old saved clusters. save the results after:
            self._blue_super_clusters            = self._combine_superClusters(new_blue_super_clusters         , self._blue_super_clusters)
            self._yellow_super_clusters          = self._combine_superClusters(new_yellow_super_clusters       , self._yellow_super_clusters)
            self._orange_big_super_clusters      = self._combine_superClusters(new_orange_big_super_clusters   , self._orange_big_super_clusters)
            self._orange_small_super_clusters    = self._combine_superClusters(new_orange_small_super_clusters , self._orange_small_super_clusters)


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
        cone_array =  np.array([],dtype=object) # list of all cones; contains all types
        '''Append cones for all 4 colors:'''
        for superCluster in self._blue_super_clusters:
            cone = SuperCluster2cone(superCluster)
            cone_array = np.append( copy.deepcopy( cone_array ) , copy.deepcopy( cone ) )
            debug_print_all_list(cone_array)


        ##
        totallNumOfCones =  (len(self._orange_big_super_clusters)   +len(self._blue_super_clusters) 
                            +len(self._orange_small_super_clusters) +len(self._yellow_super_clusters)  )
        cone_array =  [None]*totallNumOfCones # list of all cones; contains all types
        ind=0
        '''Append cones for all 4 colors:'''
        for superCluster in self._blue_super_clusters:
            cone = SuperCluster2cone(superCluster)
            cone_array[ind] = copy.deepcopy( cone )   # append
            ind +=1
            debug_print_all_list(cone_array)
        for superCluster in self._yellow_super_clusters:
            cone = SuperCluster2cone(superCluster)
            cone_array.append( cone )
        for superCluster in self._orange_big_super_clusters:
            cone = SuperCluster2cone(superCluster)
            cone_array.append( cone )
        for superCluster in self._orange_small_super_clusters:
            cone = SuperCluster2cone(superCluster)
            cone_array.append( cone )
        return cone_array

def SuperCluster2cone(superCluster):
    '''Get information from cluster'''
    x       = superCluster.getX()   
    y       = superCluster.getY()  
    x_dev   = superCluster.getXDeviation()
    y_dev   = superCluster.getYDeviation()
    Type    = superCluster.getType()
    coneId  = superCluster.getId()
    '''passs information to cone:'''
    cone = copy.deepcopy( messages.state_est.StateCone )
    cone.position   = (x,y)
    cone.position_deviation = (x_dev,y_dev)
    cone.cone_id    = coneId
    cone.type       = Type
    return cone

def optics2SuperClusters(clust, cones, ConeType):
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
        newSuperCluster = SuperCluster(ConeType , weight , mean_x,mean_y , standart_deviation_x,standart_deviation_y)   
        superClusters_LIST.append(newSuperCluster)
    return superClusters_LIST

def debug_print_all_list(givenList):
    print("---")
    for element in givenList:        
        if element == None:
            pass
        else:            
            try:
                print(f"{element.position}")
            except:
                print("passed")
# missing main