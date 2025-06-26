import numpy as np

adjCloud = np.array([[0, 0, 0, 1, 1, 1, 3, 3, 3, 5, 5],
                     [2, 1, 4, 6, 4, 2, 5, 6, 2, 2, 5],
                     [4, 3, 7, 8, 4, 3, 2, 1, 4, 5, 9]], dtype=np.float64)


randTxy = np.array([[1, 2, 3, 4],
                    [5, 6, 7, 8],
                    [9, 10, 11, 12]], dtype=np.float64)

rand_txy = np.random.uniform(-1, 1, (2,1))
randTxy[[0,1], 3] += rand_txy.flatten()

# print(adjCloud)
print(randTxy[:, :3])
print(randTxy[:, 3][:, np.newaxis])
print(rand_txy)

print(np.dot(randTxy[:, :3], adjCloud))
print(np.dot(randTxy[:, :3], adjCloud) + randTxy[:, 3][:, np.newaxis])

