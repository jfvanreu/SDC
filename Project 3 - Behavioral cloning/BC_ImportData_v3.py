"""This application is a deep learning classifier that predicts the steering angle
of a vehicle based on the images"""

# Import the necessary libraries
import glob
import os.path
import cv2
import re
import matplotlib.image as mpimg
import numpy as np
import random
import matplotlib.pyplot as plt
from sklearn.utils import shuffle
import pickle
import csv


# identify the training folders
folders=glob.glob("./training-set*/")
skip_zeros=("./training-set-16/", "./training-set-19/", "./training-set-25/", \
    "./training-set-27/", "./training-set-28/", "./training-set-30/")
    
print(folders)

# initialize training and label sets.
X_train=[]
y_train=[]

for folder in folders:
    filecount=0
    #Open the CSV file into a dictionary (filename, steering angle)
    label_file=folder+'driving_log.csv'
    reader = csv.DictReader(open(label_file))
    for row in reader:
        filename=os.path.basename(row['center'])
        #Needs to find file in correct folder
        image_file=folder+'IMG/'+filename
        steering=np.float32(row['steering'])
        if (folder in skip_zeros) & (steering==0):
            print('skipped:',image_file)
        else:
            # Load images in training set. Original images are 160x320x3.
            img=mpimg.imread(image_file)
            #resize the image to 200x66x3 (was 32x16x3).
            img = cv2.resize(img, (32, 16))
            X_train.append(img)

            #Flip the image along the vertical axis and add it to set.
            rimg=cv2.flip(img,1)
            X_train.append(rimg)
            filecount=filecount+1

            #Add the label to y_train        
            y_train.append(steering)
            #Append label for flipped image too
            y_train.append(-steering)
            print('\r', '{}'.format(filecount), ' images processed in '+folder, end='')
        
# Print the size of training and label sets
print('\n',len(X_train))
print(len(y_train))

# Pick a random image and display it with its label (steering angle)
index = random.randint(0, len(X_train))
image = X_train[index]

plt.imshow(image)
plt.show()
print(y_train[index])

# Save data to file so I don't need to regenerate data over and over
#Step 6 - Save new training set to a file.
mydict={'features': X_train, 'labels': y_train}

main_dir='./'
output = open('train.p', 'wb')
pickle.dump(mydict, output)
output.close()
print('Saved training set') 