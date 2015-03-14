#!/usr/bin/env python

"""learningArea.py: File with Learing class."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

import os
import rospy
import time
import math
import cPickle as pickle
import numpy as np
import matplotlib.pyplot as plt

from sklearn import metrics
from sklearn.cluster import KMeans
from sklearn.datasets import load_digits
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler


class Learning():
    '''
    Unsupervised Learning Class:
    Accepts a feature space, where rows are instances, and columns are features.
    '''

    def __init__(self, f_space = [[]], c_book=[], g_book=[], vis=False, \
                   load_from_file=None):

        if load_from_file is not None and load_from_file != "":
            self.load(load_from_file)
        else:
            self.feature_space  = f_space
            self.code_book = c_book
            self.graphlet_book = g_book
            self.methods = {}
            self.visualise = vis


    def save(self, dir):
        print("Saving...")
        foo = {"feature_space": self.feature_space, "code_book": self.code_book, "graphlet_book": self.graphlet_book, "learning_methods": self.methods}
        filename  = os.path.join(dir, 'smartThing.p')
        print(filename)
        with open(filename, "wb") as f:
            pickle.dump(foo, f)
        print("success")

    
    def load(self, filename):
        print("Loading QSRs from", filename)
        with open(filename, "rb") as f:
            foo = pickle.load(f)
        self.methods = foo["learning_methods"]
        self.code_book = foo["code_book"]
        self.graphlet_book = foo["graphlet_book"]
        self.feature_space = foo["feature_space"]

        print "Loaded: " + repr(self.methods)
        print("success")


    def kmeans(self, k=None):
        np.random.seed(42)
        X = np.array(self.feature_space)        
        #scaler = StandardScaler()
        #scaler.fit(X)
        #X_s = scaler.transform(X)        
        data = X #_s

        if k!=None:
            (estimator, penalty) = self.kmeans_util(data, k=k)

        else:
            print "Automatically selecting k"
            for k in xrange(1, len(data)/3):
                (estimator, penalty) = self.kmeans_util(data, k) 
                if k==1: 
                    best_pen = penalty
                    best_k = 1

                if penalty < best_pen:
                    best_pen = penalty
                    best_k = k
            k = best_k
            print "k = %d has minimum inertia*penalty" %k
            
        self.methods["kmeans"] = estimator
        if self.visualise: plot_pca(data, k)
        rospy.loginfo('6. Done')



    def kmeans_util(self, data, k=None):
        n_samples, n_features = data.shape
        if self.visualise:
            print("n_samples %d, \t n_features %d, \t n_clusters %d"
                  % (n_samples, n_features, k))
            print(40 * '-')
            print('% 9s' % 'init' '         time  inertia   *Penalty')


        (estimator, pen) = self.bench_k_means(KMeans(init='k-means++', n_clusters=k, n_init=10),
                      name="k-means++", data=data, k=k)
        self.bench_k_means(KMeans(init='random', n_clusters=k, n_init=10),
                      name="random", data=data, k=k)

        # in this case the seeding of the centers is deterministic, hence we run the
        # kmeans algorithm only once with n_init=1
        pca = PCA(n_components=k).fit(data)
        self.bench_k_means(KMeans(init=pca.components_, n_clusters=k, n_init=1),
                      name="PCA-based", data=data, k=k)
        if  self.visualise: print(40 * '-')
        return (estimator, pen)


    def bench_k_means(self, estimator, name, data, k):
        t0 = time.time()
        estimator.fit(data)
        #penalty = estimator.inertia_*math.sqrt(k)
        penalty = estimator.inertia_*k
        if  self.visualise: print('% 9s   %.2fs    %i     %i'
                % (name, (time.time() - t0), estimator.inertia_, penalty))
        return (estimator, penalty)

         

        
def plot_pca(data, k):
    ###############################################################################
    # Visualize the results on PCA-reduced data
    reduced_data = PCA(n_components=2).fit_transform(data)
    kmeans = KMeans(init='k-means++', n_clusters=k, n_init=10)
    kmeans.fit(reduced_data)

    # Step size of the mesh. Decrease to increase the quality of the VQ.
    h = .02     # point in the mesh [x_min, m_max]x[y_min, y_max].

    # Plot the decision boundary. For that, we will assign a color to each
    x_min, x_max = reduced_data[:, 0].min() + 1, reduced_data[:, 0].max() - 1
    y_min, y_max = reduced_data[:, 1].min() + 1, reduced_data[:, 1].max() - 1
    xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))

    # Obtain labels for each point in mesh. Use last trained model.
    Z = kmeans.predict(np.c_[xx.ravel(), yy.ravel()])

    # Put the result into a color plot
    Z = Z.reshape(xx.shape)
    plt.figure(1)
    plt.clf()
    plt.imshow(Z, interpolation='nearest',
               extent=(xx.min(), xx.max(), yy.min(), yy.max()),
               cmap=plt.cm.Paired,
               aspect='auto', origin='lower')


    plt.plot(reduced_data[:, 0], reduced_data[:, 1], 'k.', markersize=2)
    centroids = kmeans.cluster_centers_         # Plot the centroids as a white X
    print centroids
    plt.scatter(centroids[:, 0], centroids[:, 1],
                marker='x', s=169, linewidths=3,
                color='w', zorder=10)
    plt.title('K-means clustering on the Resource Room/trajectories data (PCA-reduced)\n'
             'Centroids are marked with white cross')
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.xticks(())
    plt.yticks(())  
    plt.show()


