"""This program generates more data from an existing dataset.
We also look at the underserved classes and focus on those"""

#Step 1 - Load the given dataset and store it in X_train, y_train
# Load pickled data
import pickle

main_dir='./traffic-signs-data/'
training_file = main_dir + 'train.p'
testing_file = main_dir + 'test.p'

with open(training_file, mode='rb') as f:
    train = pickle.load(f)
    
X_train, y_train = train['features'], train['labels']

#Step 2 - Get information about the dataset

# TODO: Number of training examples
n_train = len(X_train)

# TODO: What's the shape of an traffic sign image?
image_shape = X_train[0].shape

# TODO: How many unique classes/labels there are in the dataset.
n_classes = len(set(y_train))

print("Number of training examples =", n_train)
print("Image data shape =", image_shape)
print("Number of classes =", n_classes)

#Step 3 - Analyze the way the data is distributed by classes.

import random
from collections import Counter
import matplotlib.pyplot as plt

index = random.randint(0, len(X_train))
image = X_train[index]

#plt.imshow(image)

print(y_train[index])

y_train_counter = Counter(y_train)
print('Classes in y_train', y_train_counter)

#Step 4 - Create functions that will generate new images

import cv2
import numpy as np
from sklearn.utils import shuffle

def GaussianBlur(img):
    """Applies a Gaussian Noise kernel with kernel_size @ 5"""
    kernel_size=5
    #print('Gaussian Blur processing')
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def MedianBlur(img):
    """Applies a Median Blur transformation with kernel_size @5"""
    kernel_size=5
    #print('Median Blur processing')
    return cv2.medianBlur(img,kernel_size)

def CannyAlgo(img):
    """Applies the Canny transform"""
    low_threshold=50
    high_threshold=150
    #print('Canny algo processing')
    return cv2.Canny(img, low_threshold, high_threshold) 

def Rotation(img):
    """Peform a small random rotation of the image around its center"""
    rows,cols,channels = img.shape
    #print('Rotation processing')
    angle=random.randint(-15,15)
    M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
    return cv2.warpAffine(img,M,(cols,rows))

def Brightness(img):
    """Modifies the brightness of the image"""
    #print('Brightness adjustment processing')
    #convert RGB image to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #convert it to hsv
    
    #split image into multiple channels. V is the channel that affects brightness.
    h, s, v = cv2.split(hsv)
    
    #increase v channel by random brightness
    v += random.randint(0,128)
    final_hsv = cv2.merge((h, s, v))
    return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)

def PerspectiveTrans(img):
    """Applies a Perspective transformation to the image"""
    #print('Applies a perspective transformation')
    xdelta=random.randint(0,5)
    ydelta=random.randint(0,3)
    pts1 = np.float32([[xdelta,ydelta],[30,ydelta],[xdelta,32-ydelta],[30,32-ydelta]])
    pts2 = np.float32([[0,0],[32,0],[0,32],[32,32]])

    M = cv2.getPerspectiveTransform(pts1,pts2)
    return cv2.warpPerspective(img,M,(32,32))
    
def Translation(img):
    """Applies a translation to all pixels included in image"""
    rows,cols, channels = img.shape
    transx=random.randint(0,5)
    transy=random.randint(0,5)
    M = np.float32([[1,0,transx],[0,1,transy]])
    return cv2.warpAffine(img,M,(cols,rows))

# Step 4 - Apply transformations outlined in Step 3 randomly to the existing image set.

def find_random_image(classe_id):
    '''This function randomly picks an image among all images that belong to a classe
    '''
    #identify all images that are part of the classe_id
    matches = [i for i, y in enumerate(y_train) if (y == classe_id)]
        
    #pick a random image among all the matches. We will apply a transformation on this image
    index = random.randint(0, len(matches)-1)
    image = X_train[matches[index]]

    return image

def apply_random_transformation(img):
    '''This function applies a random transformation to an image. This helps generating more data'''
    transformations={1:Rotation, 2:PerspectiveTrans, 3:Translation}
    index=random.randint(1,len(transformations))
    img=transformations[index](img)
    return img

#Generate more data such that all classes include 2250 images
for c in range(43):
    print('\nClasse:',c)
    class_count=y_train_counter[c]
    while (class_count < 2250):
        #provide some progress visibility
        pct=(class_count/2250)*100
        print('\r', '{0:.2f}'.format(pct), ' % of class ', c, ' processed', end='')
        rand_img=find_random_image(c)
        trans_img=apply_random_transformation(rand_img)
        #add the new image to the training sets (X_train and y_train)
        X_train=np.append(X_train,[trans_img],0)
        y_train=np.append(y_train, [c], 0)
        class_count=class_count+1

#Step 5 - Shuffle the new training set and verify its validity
#Now that we added a bunch of images in the order of the classes, we better shuffle the training sets randomly.
X_train, y_train = shuffle(X_train, y_train)

#Verify that X_train and y_train are now close to 100K items
print('\nXtrain now includes', X_train.shape)
print('ytrain now includes', y_train.shape)

#Step 6 - Save new training set to a file.
mydict={'features': X_train, 'labels': y_train}

main_dir='./'
output = open('ext_training_file.p', 'wb')
pickle.dump(mydict, output)
output.close()
print('Saved extended training set')