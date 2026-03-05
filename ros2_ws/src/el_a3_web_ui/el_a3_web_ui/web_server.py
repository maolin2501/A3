#!/usr/bin/env python3
"""
EL-A3 Web 服务器（Flask + SocketIO）。
提供用于机器人控制与监控的 Web 界面。
"""

import os
import sys
import time
import threading
from datetime import datetime
from typing import Dict, List, Optional

from flask import Flask, render_template, send_from_directory, jsonify, request
from flask_socketio import SocketIO, emit

# Add package path for imports
package_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, package_dir)

from sdk_bridge import get_bridge, shutdown_bridge, SDKBridge


def find_resource_dirs():
    """查找 templates 与 static 目录。"""
    # Check multiple possible locations
    possible_bases = [
        os.path.dirname(package_dir),  # Development: ros2_ws/src/el_a3_web_ui/
        os.path.join(os.path.dirname(package_dir), '..'),  # Alternative
    ]
    
    # Also check ROS2 share directory
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('el_a3_web_ui')
        possible_bases.insert(0, share_dir)
    except Exception:
        pass
    
    template_folder = None
    static_folder = None
    
    for base in possible_bases:
        t_path = os.path.join(base, 'templates')
        s_path = os.path.join(base, 'static')
        
        if os.path.isdir(t_path) and template_folder is None:
            template_folder = t_path
        if os.path.isdir(s_path) and static_folder is None:
            static_folder = s_path
        
        if template_folder and static_folder:
            break
    
    return template_folder, static_folder


template_folder, static_folder = find_resource_dirs()

# Flask app setup
app = Flask(__name__,
            template_folder=template_folder,
            static_folder=static_folder)

app.config['SECRET_KEY'] = 'el-a3-web-ui-secret'

# SocketIO setup with eventlet
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# State history for charts
state_history: List[Dict] = []
MAX_HISTORY_LENGTH = 200

# Log history
log_history: List[Dict] = []
MAX_LOG_LENGTH = 100

# SDK Bridge reference
bridge: Optional[SDKBridge] = None


def init_sdk_bridge():
    """初始化 SDK bridge。"""
    global bridge
    try:
        bridge = get_bridge()
        if bridge:
            bridge.set_state_callback(on_state_update)
            bridge.set_log_callback(on_log_message)
            add_log('SDK Bridge 已初始化', 'info')
        else:
            add_log('SDK Bridge 初始化失败', 'error')
    except Exception as e:
        add_log(f'SDK Bridge 异常：{e}', 'error')


def add_log(message: str, level: str = 'info'):
    """将日志加入历史并推送到客户端。"""
    global log_history
    log_entry = {
        'timestamp': datetime.now().strftime('%H:%M:%S'),
        'level': level,
        'message': message
    }
    log_history.append(log_entry)
    if len(log_history) > MAX_LOG_LENGTH:
        log_history = log_history[-MAX_LOG_LENGTH:]
    
    # Emit to clients
    socketio.emit('log', log_entry)


def on_state_update(state: Dict):
    """ROS2 状态更新回调。"""
    global state_history
    
    # Add to history
    state_history.append(state)
    if len(state_history) > MAX_HISTORY_LENGTH:
        state_history = state_history[-MAX_HISTORY_LENGTH:]
    
    # Emit to all connected clients
    socketio.emit('state', state)


def on_log_message(log: Dict):
    """ROS2 日志回调。"""
    add_log(log['message'], log['level'])


# ============ Flask Routes ============

@app.route('/')
def index():
    """主页。"""
    return render_template('index.html')


@app.route('/static/<path:filename>')
def serve_static(filename):
    """静态资源。"""
    return send_from_directory(app.static_folder, filename)


@app.route('/urdf/<path:filename>')
def serve_urdf(filename):
    """URDF 与网格资源。"""
    urdf_dir = os.path.join(app.static_folder, 'urdf')
    return send_from_directory(urdf_dir, filename)


@app.route('/api/state')
def api_get_state():
    """获取当前机器人状态。"""
    if bridge:
        return jsonify(bridge.get_state())
    return jsonify({'error': 'Bridge not initialized'}), 503


@app.route('/api/history')
def api_get_history():
    """获取用于图表的状态历史。"""
    return jsonify(state_history)


@app.route('/api/logs')
def api_get_logs():
    """获取日志历史。"""
    return jsonify(log_history)


@app.route('/api/joint_limits')
def api_get_joint_limits():
    """获取关节限位。"""
    if bridge:
        return jsonify(bridge.JOINT_LIMITS)
    return jsonify({
        'L1_joint': {'lower': -2.79253, 'upper': 2.79253},
        'L2_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L3_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L4_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L5_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L6_joint': {'lower': -3.14159, 'upper': 3.14159},
    })


@app.route('/api/motor_status')
def api_get_motor_status():
    """获取电机使能状态。"""
    if bridge:
        return jsonify({
            'enabled': bridge.get_motors_enabled()
        })
    return jsonify({'error': 'Bridge not initialized'}), 503


@app.route('/api/teleop_status')
def api_get_teleop_status():
    """获取 teleop 节点状态。"""
    if bridge:
        return jsonify({
            'status': bridge.get_teleop_status()
        })
    return jsonify({'error': 'Bridge not initialized'}), 503


@app.route('/api/can_interfaces')
def api_get_can_interfaces():
    """获取可用 CAN 接口列表。"""
    interfaces = SDKBridge.get_available_can_interfaces()
    current = bridge.get_can_interface() if bridge else 'can0'
    return jsonify({
        'interfaces': interfaces,
        'current': current
    })


@app.route('/api/end_effector')
def api_get_end_effector():
    """获取末端位姿。"""
    if bridge:
        return jsonify(bridge.get_end_effector())
    return jsonify({'error': 'Bridge not initialized'}), 503


@app.route('/api/sdk_info')
def api_get_sdk_info():
    """获取 SDK 版本、连接状态、臂状态。"""
    if bridge:
        return jsonify(bridge.get_sdk_info())
    return jsonify({'error': 'Bridge not initialized'}), 503


@app.route('/api/dynamics')
def api_get_dynamics():
    """获取重力矩等动力学信息。"""
    if bridge:
        return jsonify(bridge.get_dynamics())
    return jsonify({'error': 'Bridge not initialized'}), 503


# ============ Helpers ============

def _run_blocking_cmd(cmd_type: str, fn, *args):
    """Run a potentially blocking SDK call in a background thread and notify clients."""
    try:
        ok = fn(*args)
        socketio.emit('command_result', {'success': ok, 'type': cmd_type})
    except Exception as e:
        add_log(f'{cmd_type} 异常: {e}', 'error')
        socketio.emit('command_result', {'success': False, 'type': cmd_type})


# ============ SocketIO Events ============

@socketio.on('connect')
def handle_connect():
    """客户端连接事件。"""
    add_log(f'客户端已连接：{request.sid}', 'info')
    
    # Send current state
    if bridge:
        emit('state', bridge.get_state())
    
    # Send recent logs
    for log in log_history[-10:]:
        emit('log', log)


@socketio.on('disconnect')
def handle_disconnect():
    """客户端断开事件。"""
    add_log(f'客户端已断开：{request.sid}', 'info')


@socketio.on('command')
def handle_command(data):
    """处理来自客户端的控制指令。"""
    if not bridge:
        emit('error', {'message': 'SDK bridge 不可用'})
        return
    
    cmd_type = data.get('type')
    
    try:
        if cmd_type == 'set_joints':
            # Set all joint positions
            positions = data.get('positions', [])
            duration = data.get('duration', 2.0)
            success = bridge.send_joint_command(positions, duration)
            emit('command_result', {'success': success, 'type': cmd_type})
            
        elif cmd_type == 'set_single_joint':
            # Set single joint position
            joint_index = data.get('joint_index', 0)
            position = data.get('position', 0.0)
            duration = data.get('duration', 1.0)
            success = bridge.send_single_joint_command(joint_index, position, duration)
            emit('command_result', {'success': success, 'type': cmd_type})
            
        elif cmd_type == 'go_home':
            duration = data.get('duration', 3.0)
            threading.Thread(
                target=_run_blocking_cmd, args=(cmd_type, bridge.go_home, duration),
                daemon=True).start()
            emit('command_result', {'success': True, 'type': cmd_type, 'info': 'planning'})

        elif cmd_type == 'go_zero':
            threading.Thread(
                target=_run_blocking_cmd, args=(cmd_type, bridge.go_zero),
                daemon=True).start()
            emit('command_result', {'success': True, 'type': cmd_type, 'info': 'planning'})

        elif cmd_type == 'emergency_stop':
            success = bridge.emergency_stop()
            emit('command_result', {'success': success, 'type': cmd_type})

        elif cmd_type == 'set_gripper':
            angle = data.get('angle', 0.0)
            success = bridge.set_gripper(angle)
            emit('command_result', {'success': success, 'type': cmd_type})

        elif cmd_type == 'move_to_pose':
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            z = data.get('z', 0.0)
            rx = data.get('rx', 0.0)
            ry = data.get('ry', 0.0)
            rz = data.get('rz', 0.0)
            dur = data.get('duration', 2.0)
            success = bridge.move_to_pose(x, y, z, rx, ry, rz, dur)
            emit('command_result', {'success': success, 'type': cmd_type})

        elif cmd_type == 'plan_joint_goal':
            positions = data.get('positions', [])
            vel_scale = data.get('velocity_scale', 0.3)
            threading.Thread(
                target=_run_blocking_cmd, args=(cmd_type, bridge.plan_joint_goal, positions, vel_scale),
                daemon=True).start()
            emit('command_result', {'success': True, 'type': cmd_type, 'info': 'planning'})
            
        elif cmd_type == 'set_zero_torque':
            # Set zero torque mode
            enable = data.get('enable', False)
            success = bridge.set_zero_torque_mode(enable)
            emit('command_result', {'success': success, 'type': cmd_type})
        
        # ========== Motor Enable Commands ==========
        elif cmd_type == 'enable_motors':
            # Enable all motors
            success = bridge.enable_motors()
            emit('command_result', {
                'success': success, 
                'type': cmd_type,
                'motors_enabled': bridge.get_motors_enabled()
            })
            
        elif cmd_type == 'disable_motors':
            # Disable all motors
            success = bridge.disable_motors()
            emit('command_result', {
                'success': success, 
                'type': cmd_type,
                'motors_enabled': bridge.get_motors_enabled()
            })
        
        # ========== Teleop Commands ==========
        elif cmd_type == 'start_teleop':
            # Start teleop node
            success = bridge.start_teleop()
            emit('command_result', {
                'success': success, 
                'type': cmd_type,
                'teleop_status': bridge.get_teleop_status()
            })
            
        elif cmd_type == 'stop_teleop':
            # Stop teleop node
            success = bridge.stop_teleop()
            emit('command_result', {
                'success': success, 
                'type': cmd_type,
                'teleop_status': bridge.get_teleop_status()
            })
        
        # ========== CAN Interface Commands ==========
        elif cmd_type == 'set_can_interface':
            # Set CAN interface
            interface = data.get('interface', 'can0')
            success = bridge.set_can_interface(interface)
            emit('command_result', {
                'success': success, 
                'type': cmd_type,
                'can_interface': bridge.get_can_interface()
            })
            
        else:
            emit('error', {'message': f'未知命令类型：{cmd_type}'})
            
    except Exception as e:
        add_log(f'命令执行异常：{e}', 'error')
        emit('error', {'message': str(e)})


@socketio.on('get_state')
def handle_get_state():
    """处理状态请求。"""
    if bridge:
        emit('state', bridge.get_state())
    else:
        emit('error', {'message': 'SDK bridge 不可用'})


# ============ Main Entry Point ============

def main():
    """主入口。"""
    import argparse
    
    parser = argparse.ArgumentParser(description='EL-A3 Web UI Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=5000, help='Port to bind to')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    
    args = parser.parse_args()
    
    print(f"""
╔═══════════════════════════════════════════════════════════╗
║                 EL-A3 Web 控制界面                        ║
╠═══════════════════════════════════════════════════════════╣
║  服务启动于 http://{args.host}:{args.port}                      ║
║  请在浏览器打开以进入控制面板                               ║
╚═══════════════════════════════════════════════════════════╝
    """)
    
    # Initialize SDK bridge in background
    bridge_thread = threading.Thread(target=init_sdk_bridge, daemon=True)
    bridge_thread.start()
    
    # Give bridge time to initialize
    time.sleep(1.0)
    
    try:
        # Run Flask with SocketIO
        socketio.run(app, host=args.host, port=args.port, debug=args.debug, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\n正在关闭...")
    finally:
        shutdown_bridge()


if __name__ == '__main__':
    main()
