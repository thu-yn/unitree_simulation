#!/usr/bin/env python3

import rospy
from nav_msgs.srv import SaveMap
from nav_msgs.msg import OccupancyGrid
import os

class MapSaverService:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('map_saver_service')
        
        # 创建服务
        self.save_service = rospy.Service('save_map', SaveMap, self.HandleSaveMap)
        
        # 订阅地图话题
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.MapCallback)
        self.current_map = None
        
    def MapCallback(self, map_msg):
        self.current_map = map_msg
        
    def HandleSaveMap(self, req):
        if self.current_map is None:
            rospy.logerr("没有可用的地图数据")
            return False
            
        try:
            # 确保地图保存目录存在
            map_dir = os.path.expanduser('~/maps')
            if not os.path.exists(map_dir):
                os.makedirs(map_dir)
                
            # 保存地图
            map_path = os.path.join(map_dir, 'map')
            rospy.loginfo(f"保存地图到: {map_path}")
            
            return True
        except Exception as e:
            rospy.logerr(f"保存地图时出错: {str(e)}")
            return False

if __name__ == '__main__':
    try:
        server = MapSaverService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass