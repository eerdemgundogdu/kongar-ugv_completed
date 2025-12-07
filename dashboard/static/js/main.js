const socket = io();

let moveInterval = null;

socket.on('connect', () => {
    document.querySelector('.status-dot').classList.remove('disconnected');
    document.querySelector('.status-dot').classList.add('connected');
    document.getElementById('status-text').textContent = 'Connected';
    loadWaypoints();
    loadEvents();
});

socket.on('disconnect', () => {
    document.querySelector('.status-dot').classList.remove('connected');
    document.querySelector('.status-dot').classList.add('disconnected');
    document.getElementById('status-text').textContent = 'Disconnected';
});

socket.on('state_update', (data) => {
    document.getElementById('pos-x').textContent = data.x.toFixed(2) + ' m';
    document.getElementById('pos-y').textContent = data.y.toFixed(2) + ' m';
    document.getElementById('heading').textContent = (data.theta * 180 / Math.PI).toFixed(1) + '°';
    document.getElementById('lin-vel').textContent = data.linear_vel.toFixed(2) + ' m/s';
    document.getElementById('ang-vel').textContent = data.angular_vel.toFixed(2) + ' rad/s';
    document.getElementById('battery').textContent = data.battery.toFixed(0) + '%';
    document.getElementById('mission-state').textContent = data.mission_state;
});

socket.on('imu_update', (data) => {
    document.getElementById('imu-ax').textContent = data.ax?.toFixed(2) || '0.00';
    document.getElementById('imu-ay').textContent = data.ay?.toFixed(2) || '0.00';
    document.getElementById('imu-az').textContent = data.az?.toFixed(2) || '0.00';
    document.getElementById('imu-gx').textContent = data.gx?.toFixed(2) || '0.00';
    document.getElementById('imu-gy').textContent = data.gy?.toFixed(2) || '0.00';
    document.getElementById('imu-gz').textContent = data.gz?.toFixed(2) || '0.00';
});

function sendCommand(command) {
    socket.emit('user_command', { command: command });
    addLocalEvent(`Sent command: ${command}`);
}

function startMove(linear, angular) {
    stopMove();
    sendVelocity(linear, angular);
    moveInterval = setInterval(() => sendVelocity(linear, angular), 100);
}

function stopMove() {
    if (moveInterval) {
        clearInterval(moveInterval);
        moveInterval = null;
    }
    sendVelocity(0, 0);
}

function sendVelocity(linear, angular) {
    socket.emit('cmd_vel', { linear: linear, angular: angular });
}

async function loadWaypoints() {
    try {
        const response = await fetch('/api/waypoints');
        const waypoints = await response.json();
        const list = document.getElementById('waypoint-list');
        list.innerHTML = '';
        waypoints.forEach(wp => {
            const li = document.createElement('li');
            li.innerHTML = `
                <span>${wp.name} (${wp.latitude.toFixed(6)}, ${wp.longitude.toFixed(6)})</span>
                <button onclick="deleteWaypoint(${wp.id})">✕</button>
            `;
            list.appendChild(li);
        });
    } catch (e) {
        console.error('Failed to load waypoints:', e);
    }
}

async function addWaypoint() {
    const name = document.getElementById('wp-name').value;
    const lat = parseFloat(document.getElementById('wp-lat').value);
    const lon = parseFloat(document.getElementById('wp-lon').value);

    if (!name || isNaN(lat) || isNaN(lon)) {
        alert('Please fill all fields');
        return;
    }

    try {
        await fetch('/api/waypoints', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name, latitude: lat, longitude: lon })
        });
        document.getElementById('wp-name').value = '';
        document.getElementById('wp-lat').value = '';
        document.getElementById('wp-lon').value = '';
        loadWaypoints();
        addLocalEvent(`Added waypoint: ${name}`);
    } catch (e) {
        console.error('Failed to add waypoint:', e);
    }
}

async function deleteWaypoint(id) {
    try {
        await fetch(`/api/waypoints/${id}`, { method: 'DELETE' });
        loadWaypoints();
        addLocalEvent(`Deleted waypoint #${id}`);
    } catch (e) {
        console.error('Failed to delete waypoint:', e);
    }
}

async function loadEvents() {
    try {
        const response = await fetch('/api/events');
        const events = await response.json();
        const list = document.getElementById('event-list');
        list.innerHTML = '';
        events.slice(0, 20).forEach(event => {
            const li = document.createElement('li');
            li.textContent = `[${new Date(event.timestamp).toLocaleTimeString()}] ${event.message}`;
            list.appendChild(li);
        });
    } catch (e) {
        console.error('Failed to load events:', e);
    }
}

function addLocalEvent(message) {
    const list = document.getElementById('event-list');
    const li = document.createElement('li');
    li.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    list.insertBefore(li, list.firstChild);

    while (list.children.length > 20) {
        list.removeChild(list.lastChild);
    }
}

document.addEventListener('keydown', (e) => {
    switch (e.key) {
        case 'ArrowUp':
        case 'w':
            e.preventDefault();
            sendVelocity(0.3, 0);
            break;
        case 'ArrowDown':
        case 's':
            e.preventDefault();
            sendVelocity(-0.3, 0);
            break;
        case 'ArrowLeft':
        case 'a':
            e.preventDefault();
            sendVelocity(0, 0.5);
            break;
        case 'ArrowRight':
        case 'd':
            e.preventDefault();
            sendVelocity(0, -0.5);
            break;
        case ' ':
            e.preventDefault();
            stopMove();
            break;
    }
});

document.addEventListener('keyup', (e) => {
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'w', 'a', 's', 'd'].includes(e.key)) {
        stopMove();
    }
});
