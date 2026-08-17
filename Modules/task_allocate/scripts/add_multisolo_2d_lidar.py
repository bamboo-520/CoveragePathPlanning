#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
给 Prometheus Multisolo 的 uav1~uav5.sdf 自动插入 2D 激光雷达 Gazebo ROS 插件。

用法：
  python3 add_multisolo_2d_lidar.py
  python3 add_multisolo_2d_lidar.py --root /home/amov/Prometheus/Simulator/gazebo_simulator/amov_models/Multisolo

脚本会先备份：uavN.sdf.bak_task_allocate_lidar
如果文件里已经有 task_allocate_lidar_2d，则不会重复插入。
"""
import argparse
from pathlib import Path
import re

LIDAR_TEMPLATE = r'''

      <!-- task_allocate_lidar_2d: added for sensor-based obstacle-aware A* planning -->
      <sensor name="task_allocate_lidar_2d" type="ray">
        <pose>0 0 0.18 0 0 0</pose>
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>720</samples>
              <resolution>1</resolution>
              <min_angle>-3.1415926</min_angle>
              <max_angle>3.1415926</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.20</min>
            <max>60.0</max>
            <resolution>0.02</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="task_allocate_lidar_2d_ros" filename="libgazebo_ros_laser.so">
          <topicName>/{uav_name}/prometheus/sensors/2Dlidar_scan</topicName>
          <frameName>{uav_name}_task_allocate_lidar</frameName>
        </plugin>
      </sensor>
'''


def patch_sdf(sdf_path: Path, uav_name: str) -> str:
    text = sdf_path.read_text(encoding='utf-8', errors='ignore')
    if 'task_allocate_lidar_2d' in text:
        return f'SKIP  {sdf_path} already has task_allocate_lidar_2d'

    link_match = re.search(r'<link\s+name=["\'][^"\']+["\'][^>]*>', text)
    if not link_match:
        raise RuntimeError(f'No <link name=...> found in {sdf_path}')

    end_link = text.find('</link>', link_match.end())
    if end_link < 0:
        raise RuntimeError(f'No </link> found after first link in {sdf_path}')

    backup = sdf_path.with_suffix(sdf_path.suffix + '.bak_task_allocate_lidar')
    if not backup.exists():
        backup.write_text(text, encoding='utf-8')

    lidar_block = LIDAR_TEMPLATE.format(uav_name=uav_name)
    patched = text[:end_link] + lidar_block + text[end_link:]
    sdf_path.write_text(patched, encoding='utf-8')
    return f'PATCH {sdf_path}  backup={backup}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='/home/amov/Prometheus/Simulator/gazebo_simulator/amov_models/Multisolo')
    parser.add_argument('--start', type=int, default=1)
    parser.add_argument('--end', type=int, default=5)
    args = parser.parse_args()

    root = Path(args.root)
    if not root.exists():
        raise SystemExit(f'root not found: {root}')

    for i in range(args.start, args.end + 1):
        uav_name = f'uav{i}'
        sdf_path = root / uav_name / f'{uav_name}.sdf'
        if not sdf_path.exists():
            print(f'MISS  {sdf_path}')
            continue
        print(patch_sdf(sdf_path, uav_name))

    print('\nDone. Restart Gazebo, then check:')
    print('  rostopic hz /uav1/prometheus/sensors/2Dlidar_scan')
    print('  rostopic echo -n 1 /uav1/prometheus/sensors/2Dlidar_scan/ranges')


if __name__ == '__main__':
    main()
