#!/usr/bin/python3

#
#   Developer of motorcortex: Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2017-2020 VECTIONEER.
#
import motorcortex
import mcx_tracking_cam_pb2 as tracking_cam_msg
import os
import time
import json
import socket

class tc3():
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    udpPort = 8888
    def __init__(self):
        self.ip = "192.168.42.241"
        self.udp.bind((self.ip, self.udpPort))
        # Fetching server IP from config
        self.config = json.load(open('/etc/udp2cam/config.json'))
        # Creating empty object for parameter tree
        parameter_tree = motorcortex.ParameterTree()
        # Loading protobuf types and hashes
        motorcortex_types = motorcortex.MessageTypes()
        # Open request connection
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.req, self.sub = motorcortex.connect("ws://"+self.ip+":5558:5557", motorcortex_types, parameter_tree,
                                    certificate=dir_path+"/motorcortex.crt", timeout_ms=1000,
                                    login="root", password="vectioneer")
        
        self.subscription2 = self.sub.subscribe(["root/Processing/BlobDetector/blobBuffer"], "blob", 1)
        self.subscription2.get()
        self.subscription2.notify(self.onBlob)

    def onBlob(self,val):
        try:
            blobs = tracking_cam_msg.Blobs()
            if blobs.ParseFromString(val[0].value):
                self.send_blobs_to_udp(blobs.value)
        except Exception as e:
            print(e)

    def send_blobs_to_udp(self,blobs):
        self.udp.sendto(str.encode(f"x:{blobs[0].cx}:y:{blobs[0].cy}"), (self.config["server_address"], self.udpPort))

if __name__ == '__main__':
    tc3_ex = tc3()
    while(True):
        try:
            time.sleep(1)
        except KeyboardInterrupt as e:
            print("Interrupted. Exiting.")
            tc3_ex.req.close()
            tc3_ex.sub.close()
            break
    exit()