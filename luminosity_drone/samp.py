from scipy.cluster.hierarchy import linkage, fcluster, dendrogram
from collections import Counter
import numpy as np
# List of led_led_centroids
led_centroids = np.array([(121.41807909604519, 56.4180790960452), (125.47412008281573, 24.525879917184263), (129.5, 446.5), 
             (133.5, 416.5), (145.55897435897435, 46.799145299145295), (166.0, 63.0), (170.97968936678612, 33.09557945041816), 
             (174.41807909604518, 450.58192090395477), (178.0, 419.11270983213427), (429.1597122302158, 423.2565947242206), 
             (429.77416313059877, 70.26591230551627), (443.11547619047616, 50.89523809523809), 
             (456.5829244357213, 423.5639515865227), (456.25685931115004, 73.03619381202569)])

# Define the distance threshold
threshold_distance = 300.0

# Perform hierarchical clustering
linkage_matrix = linkage(led_centroids, method='ward', metric='euclidean')

# Assign clusters based on the distance threshold
labels = fcluster(linkage_matrix, t=threshold_distance, criterion='distance')
clusters ={f"cluster_{i}": (led_centroids[labels == i]).tolist() for i in np.unique(labels)}
print(clusters)
# Print the cluster each LED belongs to
# for i, label in enumerate(labels):

#     print(f"LED #{i+1} belongs to cluster {label}")

# # Count the number of LEDs in each cluster
# counts = Counter(labels)

# # Print the count of LEDs in each cluster
# for cluster, count in counts.items():
#     print(f"Cluster #{cluster} has {count} LEDs")

for name,value in clusters.items():
    clusters[name]={
        "values":value,
        "centroid":np.mean(value,axis=0).tolist(),
        "count":len(value)
    }
    if clusters[name]["count"]==2:
        clusters[name]["type"]="alien_a"
        # clusters[name]["centroid2"]=[(clusters[name]["values"][0][0])+(clusters[name]["values"][1][0])/len(clusters[name]["values"]), (clusters[name]["values"][0][1])+(clusters[name]["values"][1][1])/len(clusters[name]["values"])]

    elif clusters[name]["count"]==3:
        clusters[name]["type"]="alien_b"
    elif clusters[name]["count"]==4:
        clusters[name]["type"]="alien_c"
    elif clusters[name]["count"]==5:
        clusters[name]["type"]="alien_d"

for name,value in clusters.items():
    print(name,value)
# print(clusters)