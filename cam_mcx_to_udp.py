#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2017-2020 VECTIONEER.
#

import motorcortex
import time
import mcx_tracking_cam_pb2 as tracking_cam_msg
from aruco_msgs.msg import Marker, MarkerArray
from blobs_msgs.msg import Blob, BlobArray
from circle_msgs.msg import CircleArray
from line_msgs.msg import LineArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import os
import rospy
import tf
from math import cos, sin, sqrt
import numpy as np

class tc3():
    def __init__(self):
        rospy.init_node('motorcortex_proxy')
        if rospy.has_param("~camera_ip"):
            self.ip = rospy.get_param("~camera_ip")
            print("get ip param:"+str(self.ip))
        else:
            self.ip = '192.168.42.1'
            print("no ip param -> use default ip")
        if rospy.has_param("~camera_frame"):
            self.frame = rospy.get_param("~camera_frame")
            print("get frame param:"+self.frame)
        else:
            self.frame = 'tc3'
            print("no frame param -> use default frame")
        
        if rospy.has_param("~namespace"):
            self.namespace = rospy.get_param("~namespace")
            print("get namespace param:"+self.namespace)
        else:
            self.self.namespace = 'tc3'
            print("no namespace param -> use default namespace")
        
        if rospy.has_param("~publish_image"):
            self.publish_image = rospy.get_param("~publish_image")
            print("get publish_image param:"+str(self.publish_image))
        else:
            self.publish_image = False
            print("no publish_image param -> use default publish_image")
        
        # Creating empty object for parameter tree
        parameter_tree = motorcortex.ParameterTree()
        # Loading protobuf types and hashes
        motorcortex_types = motorcortex.MessageTypes()
        # Open request connection
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.req, self.sub = motorcortex.connect("ws://"+self.ip+":5558:5557", motorcortex_types, parameter_tree,
                                    certificate=dir_path+"/motorcortex.crt", timeout_ms=1000,
                                    login="root", password="vectioneer")

        
        self.MarkersMarkers = tracking_cam_msg.Markers
        self.BlobsBlobs = tracking_cam_msg.Blobs
        self.aruco_tf = tf.TransformBroadcaster()
        self.pub_aruco = rospy.Publisher(self.namespace+'/markers', MarkerArray, queue_size=1)
        self.pub_blobs = rospy.Publisher(self.namespace+'/blobs', BlobArray, queue_size=1)
        self.pub_new_blobs = rospy.Publisher(self.namespace+'/new_blobs', BlobArray, queue_size=1)
        self.pub_lines = rospy.Publisher(self.namespace+'/lines', LineArray, queue_size=1)
        self.pub_circles = rospy.Publisher(self.namespace+'/circles', CircleArray, queue_size=1)
        if(self.publish_image == True):
            self.pub_image = rospy.Publisher(self.namespace+'/image', Image, queue_size=1)
            self.pub_compressed_image = rospy.Publisher(self.namespace+'/compressed_image', CompressedImage, queue_size=1)
        self.bridge = CvBridge()
        self.ip = "192.168.42.1"
        self.frame = "tracking_cam3"
        
        self.subscription2 = self.sub.subscribe(["root/Processing/BlobDetector/blobBuffer"], "blob", 1)
        self.subscription2.get()
        self.subscription2.notify(self.onBlob)

        self.subscription1 = self.sub.subscribe(["root/Processing/ArucoDetector/markerBuffer"], "marker", 1)
        self.subscription1.get()
        self.subscription1.notify(self.onMarker)

        self.subscription3 = self.sub.subscribe(["root/Processing/CircleDetector/markerCircle"], "circle", 1)
        self.subscription3.get()
        self.subscription3.notify(self.onCircle)
        
        self.subscription4 = self.sub.subscribe(["root/Processing/BlobDetectorNew/blobBuffer"], "blob", 1)
        self.subscription4.get()
        self.subscription4.notify(self.onBlobNew)

        self.subscription5 = self.sub.subscribe(["root/Processing/ProjectionModule/projectionBuffer"], "projection", 1)
        self.subscription5.get()
        self.subscription5.notify(self.onProjection)
        if(self.publish_image == True):
            self.subscription6 = self.sub.subscribe(["root/Comm_task/utilization_max","root/Processing/image"], "camera", 1)
            self.subscription6.get()
            self.subscription6.notify(self.onImage)
    
    def onLog(self,val):
        print(val[0].value)


    def onError(self,val):
        try:
            errors = motorcortex.ErrorList()
            if errors.ParseFromString(val[0].value):
                print(errors)
        except Exception as e:
            print(e)

    def onProjection(self,val):
        try:
            projection = tracking_cam_msg.Projection()
            if projection.ParseFromString(val[0].value):
                print(projection)
        except Exception as e:
            print(e)

    def onBlob(self,val):
        print("find blob")
        try:
            blobs = tracking_cam_msg.Blobs()
            
            if blobs.ParseFromString(val[0].value):
                self.send_blobs_to_ros(blobs.value)
        except Exception as e:
            print(e)

    def onBlobNew(self,val):
        # print("find new blob")
        try:
            blobs = tracking_cam_msg.Blobs()
            if blobs.ParseFromString(val[0].value):
                self.send_new_blobs_to_ros(blobs.value)
        except Exception as e:
            print(e)

    def onMarker(self,val):
        print("find marker")
        try:
            markers = tracking_cam_msg.Markers()
            if markers.ParseFromString(val[0].value):
                self.send_markers_to_ros(markers.value)
        except Exception as e:
            print(e)

    def onCircle(self,val):
        print("find circle")
        try:
            circles = tracking_cam_msg.Circles()
            if circles.ParseFromString(val[0].value):
                # print(circles.value)
                self.send_circles_to_ros(circles.value)
        except Exception as e:
            print(e)

    def onLines(self,val):
        print("find line")
        try:
            lines = tracking_cam_msg.Lines()
            if lines.ParseFromString(val[0].value):
                self.send_lines_to_ros(lines.value)
        except Exception as e:
            print(e)

    def send_blobs_to_ros(self,blobs):
        msg_array = BlobArray()
        msg_array.header.stamp = rospy.Time.now()
        msg_array.header.frame_id = self.frame
        for blob in blobs:
            msg = Blob()
            msg.id = blob.id
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame
            msg.pose.x = float(blob.cx)
            msg.pose.y = float(blob.cy)
            msg_array.blobs.append(msg)
        self.pub_new_blobs.publish(msg_array)

    def send_new_blobs_to_ros(self,blobs):
        msg_array = BlobArray()
        msg_array.header.stamp = rospy.Time.now()
        msg_array.header.frame_id = self.frame
        for blob in blobs:
            msg = Blob()
            msg.id = blob.id
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame
            msg.pose.x = float(blob.cx)
            msg.pose.y = float(blob.cy)
            msg_array.blobs.append(msg)
        self.pub_new_blobs.publish(msg_array)

    def send_circles_to_ros(self,circles):
        msg_array = CircleArray()
        msg_array.header.stamp = rospy.Time.now()
        msg_array.header.frame_id = self.frame
        for circle in circles:
            msg = Circle()
            msg.center.x = float(circle.x)
            msg.center.y = float(circle.y)
            msg.radius = float(circle.r)
            msg_array.circles.append(msg)
        self.pub_circles.publish(msg_array)


    def send_lines_to_ros(self,lines):
        msg_array = LineArray()
        msg_array.header.stamp = rospy.Time.now()
        msg_array.header.frame_id = self.frame
        for line in lines:
            msg = Line()
            msg.first.x = float(line.x0)
            msg.first.y = float(line.y0)
            msg.second.x = float(line.x1)
            msg.second.y = float(line.y1)
            msg_array.lines.append(msg)
        self.pub_lines.publish(msg_array)


    def send_markers_to_ros(self,markers):
        msg_array = MarkerArray()
        msg_array.header.stamp = rospy.Time.now()
        msg_array.header.frame_id = self.frame
        for marker in markers:
            msg = Marker()
            msg.id = marker.id
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame
            msg.pose.pose.position.x = marker.transVec3[0]
            msg.pose.pose.position.y = marker.transVec3[1]
            msg.pose.pose.position.z = marker.transVec3[2]
            ax = marker.rotVec3[0]
            ay = marker.rotVec3[1]
            az = marker.rotVec3[2]
            angle = sqrt(ax*ax + ay*ay + az*az)
            cosa = cos(angle*0.5)
            sina = sin(angle*0.5)
            msg.pose.pose.orientation.x = ax*sina/angle
            msg.pose.pose.orientation.y = ay*sina/angle
            msg.pose.pose.orientation.z = az*sina/angle
            msg.pose.pose.orientation.w = cosa
            msg_array.markers.append(msg)
            self.aruco_tf.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                                (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                                msg.header.stamp, "marker_"+str(msg.id), self.frame)
        self.pub_aruco.publish(msg_array)
    
    def onImage(self,val):
        image = cv2.imdecode(np.frombuffer(val[1].value, np.uint8), -1)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        self.pub_compressed_image.publish(self.bridge.cv2_to_compressed_imgmsg(image))
if __name__ == '__main__':
    tc3_ex = tc3()
    while not rospy.is_shutdown():
        try:
            rospy.sleep(0.1)
        except Exception as e:
            print(e)
            tc3_ex.req.close()
            tc3_ex.sub.close()