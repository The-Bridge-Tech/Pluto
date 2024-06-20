# Matthew Lauriault
# Detect line attributes from an image
# 6/19/24


# IMPORTS
import os
import cv2
import numpy as np


class Line:
        """Struct to hold line attributes."""
        def __init__(self, rho: float, theta: float):
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                self.x1 = int(x0 + 1000 * (-b))
                self.y1 = int(y0 + 1000 * (a))
                self.x2 = int(x0 - 1000 * (-b))
                self.y2 = int(y0 - 1000 * (a))
                self.theta = theta
        def getDegrees(self):
                return np.degrees(self.theta)
        def getCoord1(self) -> tuple[int, int]:
                return (self.x1, self.y1)
        def getCoord2(self) -> tuple[int, int]:
                return (self.x2, self.y2)
        def divideImage(self, image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
                """Divide image into regions based on line. Return the divided image regions."""
                # Create left and right masks to divide the image
                height, width = image.shape[:2]
                left_mask = np.zeros((height, width), dtype=np.uint8)
                right_mask = np.zeros((height, width), dtype=np.uint8)
                # Black out the respective side of the line using the respective mask
                cv2.fillPoly(left_mask, [np.array([
                    (0, 0),
                    self.getCoord1(),
                    self.getCoord2(),
                    (0, height)
                ])], 255)
                cv2.fillPoly(right_mask, [np.array([
                    (width, 0),
                    self.getCoord1(),
                    self.getCoord2(),
                    (width, height)
                ])], 255)
                # Get the left and right regions using the respective masks on the image
                left_region = cv2.bitwise_and(image, image, mask=left_mask)
                right_region = cv2.bitwise_and(image, image, mask=right_mask)
                return left_region, right_region
        def __str__(self) -> str:
                return f"Line: {self.getCoord1()}, {self.getCoord2()}, {self.getDegrees()}°"
        def __repr__(self) -> str:
                return self.__str__()

        
def detectLines(image: np.ndarray, threshold: int):
        """Detect lines from numpy image array using the given threshold. Return the lines detected."""
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply a Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Get the significant edges from the image
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        # Get the lines from the edges
        lines = cv2.HoughLines(edges, 1, np.pi/180, threshold)
        return lines


def detectLine(image: np.ndarray) -> Line:
        """Detect line from numpy image array. Returns None on failure."""
        # Perform a binary search on the threshold
        min_threshold = 100
        max_threshold = 900
        lines = None
        max_i = 1000
        i = 0
        while min_threshold <= max_threshold and i < max_i:
                mid_threshold = (min_threshold + max_threshold) // 2
                print(f"Trying threshold = {mid_threshold}")
                lines = detectLines(image, mid_threshold)
                num_lines = len(lines) if lines is not None else 0
                print(f"{num_lines} lines detected")
                # if one line was detected
                if num_lines == 1:
                        break
                # if multiple lines were detected
                elif num_lines > 1:
                        # increase min threshold
                        min_threshold = mid_threshold + 1
                # if no lines were detected
                else:
                        # decrease max threshold
                        max_threshold = mid_threshold - 1
                i += 1

        # Return the first line
        rho, theta = lines[0][0]
        return Line(rho, theta)



# TESTS

TEST_IMAGES_DIR = "src/Pluto/realsense_camera/realsense_camera/tests/line_detection"
loadTestImage = lambda img_filename: cv2.imread(os.path.join(TEST_IMAGES_DIR, img_filename))

def testDetectLine():
        # print(os.listdir(TEST_IMAGES_DIR))
        lines = {}
        for img_filename in os.listdir(TEST_IMAGES_DIR):
                image = loadTestImage(img_filename)
                line = detectLine(image)
                lines[img_filename] = line
        for img_filename, line in lines.items():
                print(f"{img_filename}: \t{line}")
                

def testDivideImage(img_filename: str):
        image = loadTestImage(img_filename)
        line = detectLine(image)
        print(line)
        left, right = line.divideImage(image)
        # save the regions as JPG files
        cv2.imwrite(os.path.join(TEST_IMAGES_DIR, f"{img_filename}_left.jpg"), left)
        cv2.imwrite(os.path.join(TEST_IMAGES_DIR, f"{img_filename}_right.jpg"), right)

def testDetectedLines(img_filename: str, threshold: int = 400):
        image = loadTestImage(img_filename)
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply a Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Get the significant edges from the image
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)
        # Apply morphological operations
        # kernel = np.ones((5, 5), np.uint8)
        # edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        # Get the lines from the edges
        lines = cv2.HoughLines(edges, 1, np.pi/180, threshold)
        if lines is not None:
                print(f"{len(lines)} lines detected in {img_filename}")
                for line in lines:
                        rho, theta = line[0]
                        a = np.cos(theta)
                        b = np.sin(theta)
                        x0 = a * rho
                        y0 = b * rho
                        x1 = int(x0 + 1000 * (-b))
                        y1 = int(y0 + 1000 * (a))
                        x2 = int(x0 - 1000 * (-b))
                        y2 = int(y0 - 1000 * (a))
                        # Draw the line on the original image
                        cv2.line(blurred, (x1, y1), (x2, y2), (0, 0, 255), 2)
        else:
                print(f"no lines detected in {img_filename}")
        # Save image with lines drawn on it as JPG file
        cv2.imwrite(os.path.join(TEST_IMAGES_DIR, f"{img_filename}_lines.jpg"), blurred)


# If this file is run as a script
if __name__ == '__main__':
        # testDetectLine()
        testDivideImage("diagonal5.jpg")
        # while True:
                # testDetectedLines("diagonal1.jpg", int(input("threshold: ")))
                # WITH BLURRING
                # Test File     Threshold       Line Accuracy           Reason
                # diagonal1     335             perfect
                # diagonal2     589             bad                     could be due to watermark
                # diagonal3     195             good
                # diagonal4     260             good
                # diagonal5                     bad






