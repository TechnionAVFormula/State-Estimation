from sklearn.cluster import OPTICS, cluster_optics_dbscan
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

IS_PLOT_RESULTS = False

def ClusteringOptics(X  , threshold):
    clust = OPTICS(min_samples=threshold, xi=.05, min_cluster_size=.05)
    clust.fit(X)

    #space = np.arange(len(X)) 
    #reachability = clust.reachability_[clust.ordering_]
    #labels = clust.labels_[clust.ordering_]

    if IS_PLOT_RESULTS :
        # plot results
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

    return clust


def _generate_test():
    
    np.random.seed(0)
    n_points_per_cluster = 20

    C1 = [-5, -2] + .8 * np.random.randn(n_points_per_cluster, 2)
    C2 = [4, -1] + .1 * np.random.randn(n_points_per_cluster, 2)
    C3 = [1, -2] + .2 * np.random.randn(n_points_per_cluster, 2)
    C4 = [-2, 3] + .3 * np.random.randn(n_points_per_cluster, 2)
    C5 = [3, -2] + 1.6 * np.random.randn(n_points_per_cluster, 2)
    C6 = [5, 6] + 2 * np.random.randn(n_points_per_cluster, 2)
    X = np.vstack((C1, C2, C3, C4, C5, C6))

    return X


if __name__ == '__main__':
    X = _generate_test()
    minSamplesNum = 4
    output = ClusteringOptics(X , minSamplesNum)


