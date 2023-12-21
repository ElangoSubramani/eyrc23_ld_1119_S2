from scipy.cluster.hierarchy import linkage, fcluster, dendrogram
from collections import Counter
import numpy as np
# List of led_led_centroids
led_centroids = np.array([
(292.0210497539639, 207.40286130854747),(377.9534760771874, 229.34716715129085)
])

# Define the distance threshold
threshold_distance = 300.0

# Perform hierarchical clustering
linkage_matrix = linkage(led_centroids, method='ward', metric='euclidean')

# Assign clusters based on the distance threshold
labels = fcluster(linkage_matrix, t=threshold_distance, criterion='distance')

# Print the cluster each LED belongs to
for i, label in enumerate(labels):
    print(f"LED #{i+1} belongs to cluster {label}")

# Count the number of LEDs in each cluster
counts = Counter(labels)

# Print the count of LEDs in each cluster
for cluster, count in counts.items():
    print(f"Cluster #{cluster} has {count} LEDs")
