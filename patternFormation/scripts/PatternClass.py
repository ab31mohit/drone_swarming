import cv2
import numpy as np
import json
import os

def findContour_COM(contour):
    moments = cv2.moments(contour)

    # Calculate the center of mass coordinates
    cX = int(moments['m10'] / moments['m00'])
    cY = int(moments['m01'] / moments['m00'])

    return (cX, cY)

##########################################
##########################################

class Pattern:

    def __init__(self, image, num_of_drones, grid_unit_metre, hover_height, path):
        self.image = image                                    # BGR image
        self.num_of_drones = num_of_drones  
        self.hover_height = hover_height                      # in meters
        self.gridSize_metre = grid_unit_metre                 # grid size in meters
        self.gridSize_pixel = None                            # Grid size in pixels
        self.origin_Pixel = [[]]                              # Pixel Coordinate of origin in the original image
        self.points_pixel_coordinates = np.zeros((self.num_of_drones, 2))
        self.points_metre_coordinates = np.zeros((self.num_of_drones, 2))
        self.grid_contour = None                              # Best grid contour
        self.pattern_contour = None                           # Best grid contour
        self.path = path
        self.img_pixels = self.image
        self.img_meters = self.image

    #############################################
    def detect_Origin(self):
        
        #convert image to gray scale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Apply binary thresholding to extract dark regions
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        # Find contours in the binary self.image
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Iterate over the contours and filter out small ones
        min_contour_area = 0.001  # Adjust this threshold as needed
        detected_lines = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_contour_area:
                detected_lines.append(contour)

        # Draw the detected lines as contours on the original image
        # cv2.drawContours(image, detected_lines, -1, (0, 0, 255), 2)

        for contour in detected_lines:

            cX, cY = findContour_COM(contour)
            self.origin_Pixel = [[cX, cY]]

        return

    ###################################################
    def pointsOnCurve(self):

        n = self.num_of_drones
        # Convert the image to HSV color space
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper green color thresholds
        lower_green = np.array([45, 100, 50])
        upper_green = np.array([75, 255, 255])

        # Create a mask for the green color
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Iterate through the contours and find the green contour with the largest area
        largest_area = 0
        green_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                green_contour = contour

        
        self.pattern_contour = green_contour

        # Approximate the green contour to make it smoother
        epsilon = 0.01 * cv2.arcLength(green_contour, True)
        approx_contour = cv2.approxPolyDP(green_contour, epsilon, True)

        # cv2.drawContours(self.image, contours, -1, (240, 210, 67), 2)

        # Calculate the perimeter of the approximated contour
        perimeter = cv2.arcLength(green_contour, True)

        # Get the total number of points on the contour
        num_points = len(contour)

        # Calculate the step size to obtain n equidistant points
        step = int(num_points / n)

        # Iterate through the contour points with the specified step size
        points_pixel_coordinates = np.zeros((n,2))

        for i in range(n):

            # Get the coordinates (x, y) of the point on the contour
            x, y = green_contour[step*i][0]
            points_pixel_coordinates[i] = np.array([x, y])

            i = i * step

        self.points_pixel_coordinates = np.round_(points_pixel_coordinates, decimals=2)

        return

    #####################################################
    def find_grid_units(self):

        # Convert the image to grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Threshold the image to obtain a binary image
        _, threshold = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)

        # Find contours in the binary image
        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Iterate through the contours
        best_grid_contour = None
        max_area = 0

        for contour in contours:

            # Calculate the perimeter of the contour
            perimeter = cv2.arcLength(contour, True)
            
            # Approximate the contour to a polygon
            epsilon = 0.001 * perimeter
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # If the polygon has 4 sides, it is a square or rectangle
            if len(approx) == 4:

                if (cv2.contourArea(contour)) > max_area :
                    best_grid_contour = contour


        self.grid_contour = best_grid_contour

        grid_perimeter = cv2.arcLength(best_grid_contour,True)
        grid_area = cv2.contourArea(best_grid_contour)
        grid_size = 4*(grid_area/grid_perimeter)

        self.gridSize_pixel = grid_size

        return

    ####################################################
    def map_pixels(self):

        # calculate the coordinates of points on curve wrt the Origin (in Pixel units of default image frame :
        # X-> left to right & Y->top to bottom)
        n = self.num_of_drones
        origin_pixel = self.origin_Pixel
        point_coordinates_pixel = self.points_pixel_coordinates
        grid_size_metre = self.gridSize_metre
        grid_size_pixel = self.gridSize_pixel

        points_wrt_origin_pixel = point_coordinates_pixel - np.full((n,2), origin_pixel)

        # Convert the above coordinates in metres
        point_wrt_origin_metre = points_wrt_origin_pixel*(grid_size_metre/grid_size_pixel)

        # In image frame, X increases from top left to top right and Y increases from top left to bottom left
        # So to map the image frame with the cartesian coordinate system, we have to rotate the points in image frame by 90* anti-clockwise
        # to ensure this, we can take the Mirror image of points wrt X axis of image frame
        # so we will take negative of the original y coordinate value for all points
        for i in range(self.num_of_drones):
            point_wrt_origin_metre[i][1] *=(-1)
        
        self.points_metre_coordinates = np.round_(point_wrt_origin_metre, decimals = 2)

        return

    ############################################
    def generate_points(self):
        self.detect_Origin()
        self.pointsOnCurve()
        self.find_grid_units()
        self.map_pixels()
        self.drawIMG_Pixels()
        self.drawIMG_Metres()

        return

    ############################################
    def savePattern_data(self):

        file_name = input("Enter the json file name (with extension) to save the Pattern Data:\n") 

        file_path = os.path.join(self.path, file_name)

        points = self.points_metre_coordinates

        # Loop to append data for each row
        for i in range(1, points.shape[0]+1):
            drone_data = {
                "x": points[i-1][0],
                "y": points[i-1][1],
                "z": self.hover_height
            }
            key = "drone" + str(i)
            data = {key: drone_data}

            # Read existing JSON data
            if os.path.exists(file_path):
                with open(file_path, "r") as json_file:
                    existing_data = json.load(json_file)
            else:
                existing_data = {}

            # Append new data to existing data
            existing_data.update(data)

            # Write updated data to JSON file
            with open(file_path, "w") as json_file:
                json.dump(existing_data, json_file)
                

        string = f'Pattern data saved to location {file_path} successfully!'
        print(string)

        return

    #################################################
    def show_IMG(self):
        img1 = self.image
        img2 = self.img_pixels
        img3 = self.img_meters

        cv2.imshow("original image", img1)
        cv2.imshow("Highlighting the pattern points with pixel units", img2)
        cv2.imshow("HIghlighting the pattern points with meter units", img3) 

        PixelIMG_path = os.path.join(self.path, 'PixelDataIMG.png')
        MeterIMG_path = os.path.join(self.path, 'RealDataIMg.png')

        cv2.imwrite(PixelIMG_path, img2)
        cv2.imwrite(MeterIMG_path, img3)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return

    ###################################################   
    def drawIMG_Pixels(self):

        img_show = self.image.copy()

        ## highlighting origin
        cX, cY = self.origin_Pixel[0]

        cv2.circle(img_show, (cX, cY), 4, (0, 0, 0), -1)
        string = f'Origin ({cX},{cY})'
        cv2.putText(img_show, string, (cX+4, cY+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, -1)

        ## highlighting grid
        cv2.drawContours(img_show, self.grid_contour, -1, (0, 0, 0), 2)
        cXgrid, cYgrid = findContour_COM(self.grid_contour)
        size = round(self.gridSize_pixel)
        string = f'{size}X{size} Grid'
        cv2.putText(img_show, string, (cXgrid + 30 -int(self.gridSize_pixel/2), cYgrid), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 51, 255), 2, -1)

        ## highlighting the pattern & points on pattern
        cv2.drawContours(img_show, self.pattern_contour, -1, (240, 210, 67), 2)
        point_num = 1

        for i in range(self.num_of_drones):

            point = self.points_pixel_coordinates[i]
            x, y = int(point[0]), int(point[1])
            image = cv2.circle(img_show, (x, y), 5, (0, 0, 0), -1)
            string = f'Point {point_num} ({x},{y})'
            cv2.putText(img_show, string, (x-80, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, -1)

            point_num += 1

        self.img_pixels = img_show

        return

    ####################################################
    def drawIMG_Metres(self):

        img_show = self.image.copy()

        ## highlighting origin
        cX, cY = self.origin_Pixel[0]

        cv2.circle(img_show, (cX, cY), 4, (0, 0, 0), -1)
        string = f'Origin (0, 0)'
        cv2.putText(img_show, string, (cX+4, cY+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, -1)

        ## highlighting grid
        cv2.drawContours(img_show, self.grid_contour, -1, (0, 0, 0), 2)
        cXgrid, cYgrid = findContour_COM(self.grid_contour)
        string = f'{self.gridSize_metre}X{self.gridSize_metre} Grid'
        cv2.putText(img_show, string, (cXgrid + 30 -int(self.gridSize_pixel/2), cYgrid), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 51, 255), 2, -1)

        ## highlighting the pattern & points on pattern
        cv2.drawContours(img_show, self.pattern_contour, -1, (240, 210, 67), 2)
        point_num = 1

        for i in range(self.num_of_drones):

            pixel = self.points_pixel_coordinates[i]
            meter = self.points_metre_coordinates[i]
            x_pixel, y_pixel = int(pixel[0]), int(pixel[1])
            x_metre, y_metre = meter[0], meter[1]
            cv2.circle(img_show, (x_pixel, y_pixel), 5, (0, 0, 0), -1)
            string = f'Point {point_num} ({x_metre},{y_metre})'
            cv2.putText(img_show, string, (x_pixel-80, y_pixel-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, -1)

            point_num += 1

        self.img_meters = img_show

        return
