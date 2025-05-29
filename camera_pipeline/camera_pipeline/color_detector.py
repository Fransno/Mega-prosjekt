import rclpy
import json
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np 

# Laster inn parametere fra config.json
with open('config.json') as file:
    data = json.load(file)

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector')

        # HSV-grenser for hver farge 
        self.red_lower1 = np.array(data["color_boundary"]["red_lower"])
        self.red_upper1 = np.array(data["color_boundary"]["red_upper"])
        self.red_lower2 = np.array(data["color_boundary"]["red2_lower"])
        self.red_upper2 = np.array(data["color_boundary"]["red2_upper"])
        self.yellow_lower = np.array(data["color_boundary"]["yellow_lower"])
        self.yellow_upper = np.array(data["color_boundary"]["yellow_upper"])
        self.blue_lower = np.array(data["color_boundary"]["blue_lower"])
        self.blue_upper = np.array(data["color_boundary"]["blue_upper"])
        self.green_lower = np.array(data["color_boundary"]["green_lower"])
        self.green_upper = np.array(data["color_boundary"]["green_upper"])
        self.contour_min_area_ = np.array(data["contour_min_area"])

        # Abonnerer på råbilde fra kamera
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)


        # Publiserer resultater og feilsøkingsbilder
        self.mask_publisher_red = self.create_publisher(Image, 'image_mask_red', 10)
        self.mask_publisher_yellow = self.create_publisher(Image, 'image_mask_yellow', 10)
        self.mask_publisher_blue = self.create_publisher(Image, 'image_mask_blue', 10)
        self.mask_publisher_green = self.create_publisher(Image, 'image_mask_green', 10)
        self.annotated_image_publisher = self.create_publisher(Image, 'image_annotated_detections', 10)
        self.detections_publisher = self.create_publisher(Float64MultiArray, 'detected_cubes', 10)
        self.isolated_image_publisher = self.create_publisher(Image, 'image_isolated_colors', 10)

        self.bridge = CvBridge() # Konvertering mellom ROS Image og OpenCV


    def image_callback(self, msg):
        # Konverter ROS Image til OpenCV BGR-format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Lag en kopi for annotering (for publisering av feilsøkingsbilde)
        annotated_image = cv_image.copy()

        # Konverter til HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Hovedarray for å samle koordinater
        main_array = []

        # Bygge opp detected_cubes, kartlegging mellom fargenavn og koder
        detections_msg = Float64MultiArray()
        color_codes = {
            "red": 1.0,
            "yellow": 2.0,
            "blue": 3.0,
            "green": 4.0
        }
        data = []
        
        
        # Prosessering for RØD

        # Lager to masker for rød farge siden rød ligger i begge ender av hue-aksen i HSV
        mask_r1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
        mask_r2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)

        # Slår sammen begge maskene for å dekke hele spekteret av rød
        mask_r = cv2.bitwise_or(mask_r1, mask_r2)

        # Finne konturer i den binære masken
        contours_r, _ = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_coords = []
              
        for contour in contours_r:
            area = cv2.contourArea(contour)
            if area > self.contour_min_area_: # Filtrerer bort små konturer (støy)
                M = cv2.moments(contour) # Beregner bildemomenter for å finne sentrum
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    red_coords.append((cX, cY))

                    # Tegner konturen og sentrum på bildet for visualisering
                    cv2.drawContours(annotated_image, [contour], -1, (0, 0, 255), 2)
                    cv2.circle(annotated_image, (cX, cY), 5, (0, 0, 255), -1) # Sirkel på sentrum
        main_array.append({"color": "red", "coordinates": red_coords})

        # Prosessering for GUL
        mask_y = cv2.inRange(hsv_image, self.yellow_lower, self.yellow_upper)
        contours_y, _ = cv2.findContours(mask_y, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        yellow_coords = []
        for contour in contours_y:
            area = cv2.contourArea(contour)
            if area > self.contour_min_area_:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    yellow_coords.append((cX, cY))
                    cv2.drawContours(annotated_image, [contour], -1, (0, 255, 255), 2)
                    cv2.circle(annotated_image, (cX, cY), 5, (0, 255, 255), -1)
        main_array.append({"color": "yellow", "coordinates": yellow_coords})

        # Prosessering for BLÅ
        mask_b = cv2.inRange(hsv_image, self.blue_lower, self.blue_upper)
        contours_b, _ = cv2.findContours(mask_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        blue_coords = []
        for contour in contours_b:
            area = cv2.contourArea(contour)
            if area > self.contour_min_area_:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    blue_coords.append((cX, cY))
                    cv2.drawContours(annotated_image, [contour], -1, (255, 0, 0), 2)
                    cv2.circle(annotated_image, (cX, cY), 5, (255, 0, 0), -1)
        main_array.append({"color": "blue", "coordinates": blue_coords})

        # Prosessering for GRØNN
        mask_g = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        contours_g, _ = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        green_coords = []
        for contour in contours_g:
            area = cv2.contourArea(contour)
            if area > self.contour_min_area_:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    green_coords.append((cX, cY))
                    cv2.drawContours(annotated_image, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(annotated_image, (cX, cY), 5, (0, 255, 0), -1)
        main_array.append({"color": "green", "coordinates": green_coords})

        # Formaterer utdata for publisering
        for color_dict in main_array:
            color = color_dict["color"]
            for (x, y) in color_dict["coordinates"]:
                data.extend([color_codes[color], float(x), float(y)])
        detections_msg.data = data

        # Lager spotlight-effekt (dim bilde utenom fargene)
        combined_mask = cv2.bitwise_or(mask_r, mask_y)
        combined_mask = cv2.bitwise_or(combined_mask, mask_b)
        combined_mask = cv2.bitwise_or(combined_mask, mask_g)
        hsv_dim = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_dim[:, :, 2] = hsv_dim[:, :, 2] // 3
        dimmed_image = cv2.cvtColor(hsv_dim, cv2.COLOR_HSV2BGR)
        dimmed_image[combined_mask != 0] = cv_image[combined_mask != 0]
        isolated_image = dimmed_image

        # Publiser resultater
        self.detections_publisher.publish(detections_msg)
        self.mask_publisher_red.publish(self.bridge.cv2_to_imgmsg(mask_r, "mono8"))
        self.mask_publisher_yellow.publish(self.bridge.cv2_to_imgmsg(mask_y, "mono8"))
        self.mask_publisher_blue.publish(self.bridge.cv2_to_imgmsg(mask_b, "mono8"))
        self.mask_publisher_green.publish(self.bridge.cv2_to_imgmsg(mask_g, "mono8"))
        self.annotated_image_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
        self.isolated_image_publisher.publish(self.bridge.cv2_to_imgmsg(isolated_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
