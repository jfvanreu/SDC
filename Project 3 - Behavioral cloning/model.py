"""This application is a deep learning classifier that predicts the steering angle
of a vehicle based on the images"""

# Import the necessary libraries
import glob
import cv2
import re
import matplotlib.image as mpimg
import numpy as np
import random
import matplotlib.pyplot as plt
from sklearn.utils import shuffle
from keras.models import Sequential
from keras.layers import Convolution2D
from keras.layers.core import Flatten, Dense, Activation, Dropout
from keras.layers.normalization import BatchNormalization
from keras.activations import relu, tanh
from keras.optimizers import Adam, SGD
from keras.models import model_from_json

# Load pickled data into X_train and y_train
import pickle

main_dir='./'
training_file = main_dir + 'train.p'

with open(training_file, mode='rb') as f:
    train = pickle.load(f)
    
X_train, y_train = np.array(train['features']), np.array(train['labels'])

# Print the size of training and label sets
print('\n',X_train.shape)
print(y_train.shape)

# Pick a random image and display it with its label (steering angle)
index = random.randint(0, len(X_train))
image = X_train[index]

plt.imshow(image)
plt.show()
print(y_train[index])

# shuffle the data
X_train, y_train = shuffle(X_train, y_train)

# Define model
model = Sequential()

# Normalize image using Keras.
model.add(BatchNormalization(input_shape=(16,32,3)))

# Convolutional layer 1 from (16,32,3) input to (12, 30, 12).
model.add(Convolution2D(24,3,3, border_mode='valid', input_shape=(16,32,3)))

# Relu activation layer from (10,28,24)
model.add(Activation('relu'))

# Dropout layer to reduce overfitting
model.add(Dropout(0.5))

# Convolutional layer 2 from (10,28,24) to (6,24,32)
model.add(Convolution2D(32,5,5, border_mode='valid', input_shape=(10,28,24)))

# Relu activation layer (6, 24, 24)
model.add(Activation('relu'))

# Convolution Layer 3 from (6,24,24) to (4,22,32)
model.add(Convolution2D(32,3,3, border_mode='valid', input_shape=(6,24,24)))

# Relu activation layer (4, 22, 32)
model.add(Activation('relu'))

model.add(Convolution2D(64,4,4, border_mode='valid', input_shape=(4,22,32)))

# Relu activation layer (4, 22, 32)
model.add(Activation('relu'))

# Flatten layer (input is (1,19,64) output is 1216)
model.add(Flatten())

# Fully Connected layer 1 (1,19,64 -> 1024)
model.add(Dense(1024))

# RELU activation layer
model.add(Activation('relu'))

# Fully Connected Layer 2
model.add(Dense(512))

# Relu activation layer
model.add(Activation('relu'))

model.add(Dense(1))

# Output layer
model.add(Activation('linear'))

#Compile the model with adam optimizer and MSE error.

adam_opti=Adam(lr=0.0005)
model.compile(adam_opti, loss='mean_squared_error')

#provide the training data (also split the training and data set)
history = model.fit(X_train, y_train, batch_size=256, nb_epoch=5, validation_split=0.25)

# convert model to JSON format
json_model = model.to_json()

#Save Json model to file
with open("./JFmodel.json", "w") as json_file:
    json_file.write(json_model)
print('Saved the model in json format')

# Save weights to HDF5 file
model.save_weights("./JFmodel.h5")
print("Saved weights to disk")