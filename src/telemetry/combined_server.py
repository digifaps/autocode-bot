#!/usr/bin/env python3
"""Rosbridge WebSocket server with correct ROS1 message format."""

import asyncio
import json
import websockets
import math
import time

imu_counter = 0

def get_imu_data():
    global imu_counter
    imu_counter += 1
    t = imu_counter * 0.1
    now = time.time()
    return {
        'header': {
            'seq': imu_counter,
            'stamp': {'secs': int(now), 'nsecs': int((now % 1) * 1e9)},
            'frame_id': 'imu_link'
        },
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        'orientation_covariance': [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'angular_velocity': {
            'x': math.sin(t * 2) * 0.1,
            'y': math.cos(t * 2) * 0.1,
            'z': math.sin(t * 3) * 0.05
        },
        'angular_velocity_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'linear_acceleration': {
            'x': math.sin(t) * 2.0,
            'y': math.cos(t) * 2.0,
            'z': 9.81 + math.sin(t * 0.5) * 0.5
        },
        'linear_acceleration_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

# ROS1-style message definitions
IMU_TYPEDEF = '''std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z'''

async def handler(websocket):
    print(f'[CONN] Client: {websocket.remote_address}')
    stream_task = None
    
    async def stream_data():
        print('[STREAM] Starting')
        try:
            while True:
                msg = {'op': 'publish', 'topic': '/imu/data_raw', 'msg': get_imu_data()}
                await websocket.send(json.dumps(msg))
                await asyncio.sleep(0.1)
        except Exception as e:
            print(f'[STREAM] Stopped: {e}')
    
    try:
        async for message in websocket:
            data = json.loads(message)
            op = data.get('op', '')
            
            if op == 'call_service':
                service = data.get('service', '')
                call_id = data.get('id', '')
                print(f'[SVC] {service}')
                
                response = {
                    'op': 'service_response',
                    'id': call_id,
                    'service': service,
                    'result': True,
                    'values': {
                        'topics': ['/imu/data_raw'],
                        'types': ['sensor_msgs/Imu'],
                        'typedefs_full_text': [IMU_TYPEDEF]
                    }
                }
                await websocket.send(json.dumps(response))
                
                if stream_task is None:
                    stream_task = asyncio.create_task(stream_data())
                    
            elif op == 'subscribe':
                print(f'[SUB] {data.get("topic", "")}')
                if stream_task is None:
                    stream_task = asyncio.create_task(stream_data())
                    
    except websockets.exceptions.ConnectionClosed:
        print('[CONN] Disconnected')
    finally:
        if stream_task:
            stream_task.cancel()

async def main():
    print('Rosbridge Server on ws://0.0.0.0:9090')
    async with websockets.serve(handler, '0.0.0.0', 9090):
        await asyncio.Future()

if __name__ == '__main__':
    asyncio.run(main())
