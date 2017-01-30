Project 3 - Behavioral cloning
------------------------------

Preprocessing
=============
- The images were converted to a 16x32x3 format. This is 10 times smaller than the original images.
It allowed me to run the testing on my local machine (no GPU).
- Randomly applied some changes to the brightness of images. This helped generalizing the learning.
This was only applied to the training and validation sets.

Model description
=================
My model was inspired from two sources: the NVIDIA research paper and a post on the forum 
where a student explained that he converted images to 16x32x3.

I used the Keras framework for this project.

Layer 1: Normalization layer. 
The model starts with a normalization layer so all image pixels are in the same range.
It takes an image of dimension (16,32,3) as input.

We then have 4 ConvNet layers that result into a 64 features @ (1,19) output.
We perform a dropout operation after the first ConvNet transformation to reduce overfitting:

(1) Convolutional layer 1 from (16,32,3) input to (12, 30, 12). 
    Followed by RELU activation and dropout (50%) layers.

(2) Convolutional layer 2 from (10,28,24) to (6,24,32)
    Followed by RELU activation.
    
(3) Convolution Layer from (6,24,24) to (4,22,32)
    Followed by RELU activation.

(4) Convolutional Layer from (4,22,32) to (1, 19, 64)
    Followed by RELU activation layer.

We then flatten the resulting output and use it as input to our FC layer 1 with 1024 features.
We add another dropout layer after the FC layer 1 this time. 
This dropout layer is followed by two FC layers of 512 elements and 1 element respectively.

Optimization function: We use the Adam optimizer and compute the loss through Mean Square Error formula.

Hyperparmaters:
===============

We initially performed 15 epochs, but realized that the results were not improving much as to what we experience after 5 epochs.
Our LrR was 0.0005. It seemed to progress  well towards the minimum without taking too much time.

Training and Validation Sets
============================
The samples were split into a Training Set (75% of the samples) and a Validation set (25%).
In general, both sets showed similar losses. In fact, sometimes the validation loss was even smaller.

Regularization
==============
We used multiple approaches to improve regularization:

1. We tried to use a fairly large dataset. We flipped the images and use the inverse steering
degree so the model doesn't favor the left turn - more common on track 1.

2. I used two dropout layers in the model. One after the first Convolutional Layer and 
another dropout layer after the first fully-connected layer.

3. I included a preprocessing step which randomly modifies the brightness of an image.

Areas for improvement:
======================

- Recording data would have been better with a real steering wheel. I didn't have access
to a joystick, so I used my keyboard. Unfortunately, this creates some data with 0 steering
when you release fingers from the keyboard. This makes the data less "continuous".

- The pitstop exit is still a bit wide. 
I tried to fix this multiple times but then it caused problems in other areas.
