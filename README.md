# Greenguard

#Requirement
The existing setup environment in plant nurseries poses several challenges, including wasted resources, increased maintenance efforts, and potential damage to plants. Additionally, individuals who are away from home or reside in old-age homes struggle to maintain greenery due to a lack of time and difficulties in plant care management.
In order to address these problems, a solution is needed that allows plants to thrive in old-age homes and survive when individuals are away, also addresses the issues faced by business owners in plant nurseries.  The solution should save time and energy, provide a great alternative to conventional watering systems, and prevent overwatering that damages both plants and the surrounding environment.

#Features
This code helps in identifying the plant pots in the given image and also give us the position of where exactly it is on a scale of -10 to +10.

#Future Prospects
This is just a prototype that identifies the plants in the image. In the future we plan to identify the plant in live captured video and send signal to the watering system .

#How we built it
#Step 1- Import the neccesary libraries
Libraries include-
Numpy
Matplotlib
cv2

#Step 2- Gather the training data
Training data was gathered from google images and some were captured from the surroundings.

#Step 3- Preprocess the data
Reading the images with tensorflow utils as a array of 256,256 pixels
Scaling the data by dividing it by 255
Creating the custom weights and configuration files

#Step 4- Creating the Model
We read the image we need to find the plant from.
Deep neural network(dnn) is used to identify the objects in the image
If the identified object is plant pot, it draws the bounding box with the labels and prints the image.
The position of the plant pot on the x axis is given on the scale of -10 to +10 (from left to right)
