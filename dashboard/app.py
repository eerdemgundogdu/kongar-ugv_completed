from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import database
import threading
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'k0ng4r_s3cr3t_2026'
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

database.init_db()

# In-memory state
current_state = {
    'x': 0.0,
    'y': 0.0,
    'theta': 0.0,
    'linear_vel': 0.0,
    'angular_vel': 0.0,
    'mission_state': 'INIT',
    'connected': False,
    'battery': 100.0,
    'imu': {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0}
}

# Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/state')
def get_state():
    return jsonify(current_state)

@app.route('/api/waypoints', methods=['GET'])
def get_waypoints():
    return jsonify(database.get_waypoints())

@app.route('/api/waypoints', methods=['POST'])
def add_waypoint():
    data = request.json
    wp_id = database.add_waypoint(data['name'], data['latitude'], data['longitude'])
    return jsonify({'id': wp_id, 'success': True})

@app.route('/api/waypoints/<int:wp_id>', methods=['DELETE'])
def delete_waypoint(wp_id):
    database.delete_waypoint(wp_id)
    return jsonify({'success': True})

@app.route('/api/missions', methods=['GET'])
def get_missions():
    return jsonify(database.get_missions())

@app.route('/api/missions', methods=['POST'])
def create_mission():
    data = request.json
    mission_id = database.create_mission(data.get('name', ''))
    return jsonify({'id': mission_id, 'success': True})

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    limit = request.args.get('limit', 100, type=int)
    mission_id = request.args.get('mission_id', None, type=int)
    return jsonify(database.get_telemetry(mission_id, limit))

@app.route('/api/events', methods=['GET'])
def get_events():
    return jsonify(database.get_events())

# WebSocket Events
@socketio.on('connect')
def handle_connect():
    emit('state_update', current_state)

@socketio.on('cmd_vel')
def handle_cmd_vel(data):
    linear = data.get('linear', 0.0)
    angular = data.get('angular', 0.0)
    socketio.emit('cmd_vel_to_ros', {'linear': linear, 'angular': angular})

@socketio.on('user_command')
def handle_user_command(data):
    command = data.get('command', '')
    socketio.emit('user_command_to_ros', {'command': command})
    database.add_event('command', f'User command: {command}')

@socketio.on('telemetry_update')
def handle_telemetry(data):
    current_state['x'] = data.get('x', 0.0)
    current_state['y'] = data.get('y', 0.0)
    current_state['theta'] = data.get('theta', 0.0)
    current_state['linear_vel'] = data.get('linear_vel', 0.0)
    current_state['angular_vel'] = data.get('angular_vel', 0.0)
    current_state['connected'] = True
    socketio.emit('state_update', current_state, broadcast=True)

@socketio.on('mission_state_update')
def handle_mission_state(data):
    current_state['mission_state'] = data.get('state', 'UNKNOWN')
    socketio.emit('state_update', current_state, broadcast=True)

@socketio.on('imu_update')
def handle_imu(data):
    current_state['imu'] = data
    socketio.emit('imu_update', data, broadcast=True)

if __name__ == '__main__':
    print('Starting UGV Dashboard on http://localhost:5000')
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
