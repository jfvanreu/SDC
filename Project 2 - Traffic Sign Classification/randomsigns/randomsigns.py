import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
import numpy as np
import cv2
import re

image_file_list=glob.glob("*.ppm")

test_set_img=[]
test_set_label=[]

#open traffic images found on the internet and convert them to 32x32x3 format
for image_file in image_file_list:
    img=mpimg.imread(image_file)
    #resize the image to 32x32x3
    img = cv2.resize(img, (32, 32))
    test_set_img.append(img)
    
    #Extract the label from the filename
    test_set_label.append(re.findall(r'\d+',image_file))


fig=plt.figure(figsize=(20,10))
number_of_files=len(test_set_img)

for i in range(number_of_files):
    a=fig.add_subplot(1,number_of_files,i+1)
    image = test_set_img[i]
    label = test_set_label[i]
    title="Label:"+str(label)
    a.set_title(title)
    plt.imshow(image)

plt.show()