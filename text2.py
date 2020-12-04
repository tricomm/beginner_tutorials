import scipy.io as scio

dataFile ="./image00002.mat"
data = scio.loadmat(dataFile)
print(type(data))
for key,value in data.items():
    print(key)
print(data["Pose_Para"])
