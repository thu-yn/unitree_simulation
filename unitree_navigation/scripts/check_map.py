#!/usr/bin/env python3

import rospy
import os
import subprocess

def check_map_exists(map_name):
    """检查地图文件是否存在"""
    # 获取地图文件路径
    map_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'maps', map_name)
    yaml_path = map_path + '.yaml'
    pgm_path = map_path + '.pgm'
    
    # 检查文件是否存在
    if os.path.exists(yaml_path) and os.path.exists(pgm_path):
        return True, map_path
    else:
        return False, map_path

def main():
    """主函数"""
    rospy.init_node('map_checker', anonymous=True)
    
    # 获取参数
    map_name = rospy.get_param('~map_name', 'my_map')
    robot_ip = rospy.get_param('~robot_ip', '192.168.123.220')
    
    rospy.loginfo(f"检查地图: {map_name}")
    
    # 检查地图是否存在
    map_exists, map_path = check_map_exists(map_name)
    
    if map_exists:
        rospy.loginfo(f"找到地图: {map_path}，启动导航模式")
        # 启动导航模式
        subprocess.call(['roslaunch', 'unitree_navigation', 'unitree_navigation.launch', 
                         f'map_name:={map_name}', f'robot_ip:={robot_ip}', 'use_existing_map:=true'])
    else:
        rospy.loginfo(f"未找到地图: {map_name}，启动建图模式")
        # 启动建图模式
        subprocess.call(['roslaunch', 'unitree_navigation', 'unitree_navigation.launch', 
                         f'map_name:={map_name}', f'robot_ip:={robot_ip}', 'use_existing_map:=false'])

if __name__ == "__main__":
    main() 