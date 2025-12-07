import sqlite3
import os
from datetime import datetime

DB_PATH = os.path.join(os.path.dirname(__file__), 'ugv_data.db')

def get_connection():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

def init_db():
    conn = get_connection()
    cursor = conn.cursor()
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS waypoints (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            latitude REAL NOT NULL,
            longitude REAL NOT NULL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS missions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT,
            start_time TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            end_time TIMESTAMP,
            status TEXT DEFAULT 'pending',
            notes TEXT
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            x REAL,
            y REAL,
            theta REAL,
            linear_vel REAL,
            angular_vel REAL,
            mission_id INTEGER,
            FOREIGN KEY (mission_id) REFERENCES missions(id)
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            event_type TEXT,
            message TEXT,
            mission_id INTEGER,
            FOREIGN KEY (mission_id) REFERENCES missions(id)
        )
    ''')
    
    conn.commit()
    conn.close()

# Waypoint CRUD
def get_waypoints():
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM waypoints ORDER BY id')
    rows = cursor.fetchall()
    conn.close()
    return [dict(row) for row in rows]

def add_waypoint(name, lat, lon):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('INSERT INTO waypoints (name, latitude, longitude) VALUES (?, ?, ?)', (name, lat, lon))
    conn.commit()
    wp_id = cursor.lastrowid
    conn.close()
    return wp_id

def delete_waypoint(wp_id):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('DELETE FROM waypoints WHERE id = ?', (wp_id,))
    conn.commit()
    conn.close()

# Mission CRUD
def create_mission(name=''):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('INSERT INTO missions (name, status) VALUES (?, ?)', (name, 'running'))
    conn.commit()
    mission_id = cursor.lastrowid
    conn.close()
    return mission_id

def end_mission(mission_id, status='completed'):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('UPDATE missions SET end_time = ?, status = ? WHERE id = ?', 
                   (datetime.now(), status, mission_id))
    conn.commit()
    conn.close()

def get_missions(limit=20):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM missions ORDER BY start_time DESC LIMIT ?', (limit,))
    rows = cursor.fetchall()
    conn.close()
    return [dict(row) for row in rows]

# Telemetry
def add_telemetry(x, y, theta, linear_vel, angular_vel, mission_id=None):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO telemetry (x, y, theta, linear_vel, angular_vel, mission_id)
        VALUES (?, ?, ?, ?, ?, ?)
    ''', (x, y, theta, linear_vel, angular_vel, mission_id))
    conn.commit()
    conn.close()

def get_telemetry(mission_id=None, limit=100):
    conn = get_connection()
    cursor = conn.cursor()
    if mission_id:
        cursor.execute('SELECT * FROM telemetry WHERE mission_id = ? ORDER BY timestamp DESC LIMIT ?', 
                       (mission_id, limit))
    else:
        cursor.execute('SELECT * FROM telemetry ORDER BY timestamp DESC LIMIT ?', (limit,))
    rows = cursor.fetchall()
    conn.close()
    return [dict(row) for row in rows]

# Events
def add_event(event_type, message, mission_id=None):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('INSERT INTO events (event_type, message, mission_id) VALUES (?, ?, ?)',
                   (event_type, message, mission_id))
    conn.commit()
    conn.close()

def get_events(limit=50):
    conn = get_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM events ORDER BY timestamp DESC LIMIT ?', (limit,))
    rows = cursor.fetchall()
    conn.close()
    return [dict(row) for row in rows]

if __name__ == '__main__':
    init_db()
    print(f'Database initialized at {DB_PATH}')
