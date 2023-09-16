import glob, os

image_path = '/home/ryan/catkin_ws/src/comp4034/src/assignment/darknet/images'
label_path = '/home/ryan/catkin_ws/src/comp4034/src/assignment/darknet/labels'

# Percentage of images to be used for the test set
percentage_test = 10

# Create and/or truncate train.txt and test.txt
file_train = open('train.txt', 'w')  
file_test = open('valid.txt', 'w')

# Populate train.txt and test.txt
counter = 1  
index_test = round(100 / percentage_test)  
for pathAndFilename in glob.iglob(os.path.join(label_path, "*.txt")):  
    title, ext = os.path.splitext(os.path.basename(pathAndFilename))

    if counter == index_test+1:
        counter = 1
        file_test.write(image_path + "/" + title + '.jpg' + "\n")
    else:
        file_train.write(image_path + "/" + title + '.jpg' + "\n")
        counter = counter + 1