import numpy as np
import cv2
from os import listdir
from os.path import join, isfile


class ImageSegmentation:
    def __init__(self, src_directory: str, dst_directory: str, img_range: list):
        
        '''
            Ex:
            img_range = [1, 95] -> (labels das images 1 a 95)
        '''
        self.src_directory = src_directory
        self.dst_directory = dst_directory
        self.img_range = img_range
        self.my_img_names: str = self.set_list_of_img_names(image_range = img_range)

        self.current_image = None
        self.current_mask = None

        #list of images to label
        self.src_images = []
        self.dst_images = []
        
        

        #add all images in directory to self.src_images
        for file_name in listdir(self.src_directory):
            
            #Verifica se a imagem ja possui ground thruth salvo
            if file_name not in listdir(self.dst_directory):
                
                #Verifica se a imagem esta no range selecionado
                if file_name[:-4] in self.my_img_names:
                
                    self.src_images.append(file_name)
        

        print(f'Total images: {len(self.src_images)}')
        
        self.number_of_images = len(self.src_images)

        #color range selection
        self.lower_blue = 0
        self.upper_blue = 0        
        self.lower_green = 0
        self.upper_green = 0        
        self.lower_red = 0
        self.upper_red = 0
            
    
    @staticmethod
    def set_list_of_img_names(image_range: list):
        num1 = image_range[0]
        num2 = image_range[1]
        names_str = ''

        for i in range(num1, num2):
            if i < 10:
                names_str += f'00{i} '
            
            elif 10 <= i < 100:
                names_str += f'0{i} '
            
            else:
                names_str += f'{i} '
        return names_str
        

    @staticmethod
    def nothing(x):
        pass

    def start_segmentation(self):
        
        ##Trackbars
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("Lower Blue", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("Upper Blue", "Trackbars", 0, 255, self.nothing)

        cv2.createTrackbar("Lower Green", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("Upper Green", "Trackbars", 0, 255, self.nothing)

        cv2.createTrackbar("Lower Red", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("Upper Red", "Trackbars", 0, 255, self.nothing)
        
        img_index = 0
        while True:
            
            if len(self.src_images) == 0:
                print('src image is empty')
                break

            current_file_path = self.src_directory + self.src_images[img_index]
            self.current_image = cv2.imread(filename = current_file_path)
            
            #color range selection
            self.lower_blue = cv2.getTrackbarPos("Lower Blue", "Trackbars")
            self.upper_blue = cv2.getTrackbarPos("Upper Blue", "Trackbars")
            
            self.lower_green = cv2.getTrackbarPos("Lower Green", "Trackbars")
            self.upper_green = cv2.getTrackbarPos("Upper Green", "Trackbars")
            
            self.lower_red = cv2.getTrackbarPos("Lower Red", "Trackbars")
            self.upper_red = cv2.getTrackbarPos("Upper Red", "Trackbars")

            self.current_mask = cv2.inRange(self.current_image,
                        (self.lower_blue, self.lower_green, self.lower_red),
                        (self.upper_blue, self.upper_green, self.upper_red))
            
            cv2.imshow("Current Image", self.current_image)
            cv2.imshow("Mask", self.current_mask)

            k = cv2.waitKey(1)
            if k == 27:
                break
            
            #https://stackoverflow.com/questions/14494101/using-other-keys-for-the-waitkey-function-of-opencv
            #spaceKey to save current mask
            elif k == 32:
                mask_name = self.src_images[img_index]
                save_to_path = self.dst_directory + mask_name
                
                cv2.imwrite(filename = save_to_path, img=self.current_mask)
                print(f'Image: {mask_name} | saved to path: {self.dst_directory}')

            #go to next image
            elif k == ord('e'):
                print(f'Image: {current_file_path}')
                img_index += 1
            
            #back to previous image
            elif k == ord('q'):
                print(f'Image: {current_file_path}')
                img_index -= 1
    

