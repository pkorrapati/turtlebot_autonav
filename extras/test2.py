import numpy as np

LIDAR_EDGE_DEPTH = 1.5
MIN_EDGE_WIDTH = 2

a = [1.2,2,3,10,10,2.5,2.2,3,1.8,1.9,10,10,10,10,10,10,2.5,10,2.8,2.7,3,2.4,2.6] #Example 1
# a = [10,2,3,2.5,10,2.5,2.2,3,1.8,1.9,10,10,10,10,10,10,2.5,10,2.8,2.7,3,2.4,2.6] #Example 2 where inf is at 0 indx
# a = [1.2,2,3,2.5,2.2,10,2.5,3,1.8,1.9,10,10,10,10,10,10,2.5,10,2.8,2.7,3,2.4,2.6] #Example 3 where start has > 3
a = [1.2,2,3,2.5,2.2,10,2.5,3,1.8,1.9,10,10,10,10,10,10,2.5,10,2.8,2.7,3,2.4,10] #Example 4 where inf is at last
# a = [1.2,2,3,2.5,2.2,10,2.5,3,1.8,1.9,10,10,10,10,10,10,2.5,10,10,10,10,10,2.6] #Example 5 where inf is at last-1 
# print(a)

rangeDiff = np.abs(np.diff(a))
print(rangeDiff)

cornerIndex = np.where(rangeDiff > LIDAR_EDGE_DEPTH)[0] + 1
print(cornerIndex)

edges_raw = np.split(a, cornerIndex)

print("\n Raw: \n")
for edge in edges_raw:
    print(edge)

print('\n')

shortEdges = np.unique(np.array(list((i-1,i) for i in range(len(edges_raw)) if len(edges_raw[i]) < MIN_EDGE_WIDTH)).flatten())
shortEdges = shortEdges[np.where(np.multiply(shortEdges < len(cornerIndex), shortEdges >= 0))]
print(shortEdges)

print("\n Clean Edges : \n")
cleanCorners = np.setdiff1d(cornerIndex, cornerIndex[shortEdges], assume_unique=True)

edges_clean = np.split(a, cleanCorners)

print("\n Raw: \n")
for edge in edges_clean:
    print(edge)
# print(np.setdiff1d(cornerIndex, cornerIndex[shortEdges], assume_unique=True))



# print(shortEdges[np.where(np.diff(shortEdges) == 1)[0]])

# cInd1 = np.sort(np.unique(np.append(cornerIndex, [0, len(a)])))
# print(cInd1)

# cInd1_cont = cInd1[np.where(np.diff(cInd1) > MIN_EDGE_WIDTH)[0]]
# print(cInd1_cont)

# edges_clean = np.split(a, cInd1_cont)

# print("\n Cleaned: \n")
# for edge in edges_clean:
#     print(edge)