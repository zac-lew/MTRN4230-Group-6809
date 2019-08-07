# Script to rename test images

# importing os module
import os

# Function to rename multiple files
def main():
    path = 'C:\\Users\\Jonathan\\Documents\\UNSW Engineering\\2019\\S2\\MTRN4230\\Group Project\\YOLO_TEST\\Shape_Color'
    os.chdir(path)
    i=1

    for file in os.listdir(path):
      src=file
      dst="SC"+str(i)+".jpg"
      os.rename(src,dst)
      i+=1

main()
