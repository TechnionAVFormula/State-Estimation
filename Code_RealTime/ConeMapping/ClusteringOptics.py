from sklearn.cluster import OPTICS, cluster_optics_dbscan
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

from StateEst_Utils.config import IS_PLOT_CONE_MAP_RESULTS 

IS_PLOT_RESULTS = True

def ClusteringOptics(X  , threshold):
    clust = OPTICS(min_samples=threshold, xi=.55, min_cluster_size=.05 , max_eps=3)
    if X.size == 0: # We don't have any samples of that type of cones:
        return clust #return as is: (empty Optics object)
    elif X.ndim==2 and X.shape[1]==2 and X.shape[0]>0:  # A lgeit column array, with  rows of (x,y) coordinates: 
        clust.fit(X)   
    else:
        raise Exception("Impossible array size")

    #space = np.arange(len(X)) 
    #reachability = clust.reachability_[clust.ordering_]
    #labels = clust.labels_[clust.ordering_]

    if IS_PLOT_CONE_MAP_RESULTS :
        plot_results(X , clust)
        

    return clust


def plot_results(X , clust):
    plt.figure()
    G = gridspec.GridSpec(1, 1)
    ax2 = plt.subplot(G[0, 0]) 
    colors = ['g.', 'r.', 'b.', 'y.', 'c.']
    for klass, color in zip(range(0, 5), colors):
        Xk = X[clust.labels_ == klass]
        ax2.plot(Xk[:, 0], Xk[:, 1], color, alpha=0.3)
    ax2.plot(X[clust.labels_ == -1, 0], X[clust.labels_ == -1, 1], 'k+', alpha=0.1)
    ax2.set_title('Automatic Clustering\nOPTICS')
    plt.tight_layout()
    plt.show()



if __name__ == '__main__':
    val = 42
    print(f"no input given. {val} is the answer")
