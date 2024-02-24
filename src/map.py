import rospy
from sensor_msgs.msg import NavSatFix
import googlemaps
from PIL import Image
from typing import Tuple
import io
import numpy as np
import matplotlib.pyplot as plt

client: googlemaps.Client = None
current_image: Image = None

def download_area(center: Tuple[float, float]):
    global current_image

    size = (400, 400)

    b = io.BytesIO()

    for chunk in client.static_map(size=size, center=center, zoom=15):
        if chunk:
            b.write(chunk)

    current_image = Image.open(b)
    img = np.asarray(current_image)
    plt.imshow(img)
    plt.show()


def callback(data: NavSatFix):
    if current_image is None:
        download_area((data.latitude, data.longitude))


def listener():
    global client

    rospy.init_node('map_display')
    rospy.Subscriber('/gps/filtered', NavSatFix, callback)

    key = rospy.get_param("~key")
    client = googlemaps.Client(key)

    rospy.spin()

if __name__ == '__main__':
    listener()
