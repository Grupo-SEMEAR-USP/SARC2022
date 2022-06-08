import cv2
import os

def generate_video():
    image_folder = '.' # make sure to use your folder
    video_name = 'film.mp4'
      
    images = [img for img in os.listdir(image_folder)
              if img.endswith(".jpg") and img.startswith('img')]
     
    # Array images should only consider
    # the image files ignoring others if any

    images.sort(key=lambda x: int(x.split('.')[0].split('g')[1]))
  
    frame = cv2.imread(os.path.join(image_folder, images[0]))
  
    # setting the frame width, height width
    # the width, height of first image
    height, width, layers = frame.shape  
  
    video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), 10, (width, height)) 
  
    # Appending the images to the video one by one
    for image in images: 
        video.write(cv2.imread(os.path.join(image_folder, image))) 
      
    # Deallocating memories taken for window creation
    cv2.destroyAllWindows() 
    video.release()  # releasing the video generated
  
  
# Calling the generate_video function
generate_video()