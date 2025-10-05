let combinedChart = null;

function updateCombinedChart(rssi, dist) {
    const length = rssi.length;

    // Generate time labels based on update interval (1 second)
    // Assuming data is collected every 1 second, create relative time labels
    const now = new Date();
    const labels = Array(length).fill(0).map((_, i) => {
        const secondsAgo = (length - 1 - i);
        const time = new Date(now - secondsAgo * 1000);
        return time.toLocaleTimeString('en-US', { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    });

    if (!combinedChart) {
        const ctx = document.getElementById('combinedChart').getContext('2d');
        combinedChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: labels,
                datasets: [
                    {
                        label: 'RSSI (dBm)',
                        data: rssi,
                        borderColor: '#1a368c',
                        backgroundColor: 'rgba(37,99,234,0.12)',
                        fill: false,
                        yAxisID: 'y_rssi',
                        pointRadius: 2,
                        tension: 0.1
                    },
                    {
                        label: 'Distance (m)',
                        data: dist,
                        borderColor: '#2563ea',
                        backgroundColor: 'rgba(37,99,234,0.09)',
                        fill: false,
                        yAxisID: 'y_dist',
                        pointRadius: 2,
                        tension: 0.1
                    }
                ]
            },
            options: {
                animation: {
                    duration: 200,
                },
                layout: {
                    margin: { left: 0, right: 0, top: 0, bottom: 0 },
                    padding: { left: 0, right: 0, top: 0, bottom: -20 }
                },
                responsive: true,
                scales: {
                    x: {
                        display: true,
                        title: {
                            display: true,
                        },
                        ticks: {
                            maxRotation: 20,
                            minRotation: 20,
                            maxTicksLimit: 10,
                            autoSkip: true,
                        }
                    },
                    y_rssi: {
                        type: 'linear',
                        position: 'left',
                        title: {
                            display: true,
                            text: 'RSSI (dBm)'
                        },
                        ticks: {
                            min: -150,
                            max: 0
                        }
                    },
                    y_dist: {
                        type: 'linear',
                        position: 'right',
                        title: {
                            display: true,
                            text: 'Distance (m)'
                        },
                        grid: {
                            drawOnChartArea: false
                        }
                    }
                },
                plugins: {
                    legend: {
                        display: true,
                        position: 'top'
                    }
                }
            }
        });
    } else {
        combinedChart.data.labels = labels;
        combinedChart.data.datasets[0].data = rssi;
        combinedChart.data.datasets[1].data = dist;
        combinedChart.update();
    }
}

let isFetchingStats = false;
let previousPacketsSent = 0;
let previousPacketsAcked = 0;
let lastPacketWasAcked = true;  // Track if the last packet was acknowledged

function updateStats() {
    if (isFetchingStats) return;
    isFetchingStats = true;

    fetch('stats.json')
        .then(response => response.json())
        .then(data => {
            // Update merged packet counter: acked / total (percent%)
            const successPct = (100 - data.loss_pct).toFixed(2);
            document.getElementById('packets').textContent = `${data.packets_acked} / ${data.packets_sent} (${successPct}%)`;

            document.getElementById('rssi').textContent = data.last_rssi;
            document.getElementById('distance').textContent = data.distance.toFixed(1);
            document.getElementById('gps_numSV').textContent = data.gps_numSV;
            document.getElementById('gps_lat').textContent = data.gps_lat.toFixed(7);
            document.getElementById('gps_lon').textContent = data.gps_lon.toFixed(7);
            document.getElementById('gps_fix_type').textContent = `${data.gps_fix_type} / ${data.gps_fix_ok ? 'OK' : 'NO'}`;

            // Detect if a new packet was sent and whether it was acknowledged
            if (data.packets_sent > previousPacketsSent) {
                // New packet was sent
                if (data.packets_acked > previousPacketsAcked) {
                    lastPacketWasAcked = true;  // Acknowledged
                } else {
                    lastPacketWasAcked = false; // Lost/Not acknowledged
                }
                previousPacketsSent = data.packets_sent;
                previousPacketsAcked = data.packets_acked;
            }

            updateCombinedChart(data.rssi_hist || [], data.distance_hist || []);
        })
        .catch(error => {
            console.error('Error fetching stats:', error);
        })
        .finally(() => {
            isFetchingStats = false;
        });
}

document.getElementById('resetBtn').addEventListener('click', () => {
    fetch('/reset_stats', { method: 'POST' });
});

document.getElementById('ota').addEventListener('click', () => {
    fetch('/ota', { method: 'POST' }).then(response => {
        if (response.ok) {
            document.getElementById('ota').style.backgroundColor = 'orange';
        } else {
            alert('Failed to start OTA');
        }
    });
});

document.getElementById('bootloader').addEventListener('click', () => {
    fetch('/bootloader', { method: 'POST' }).then(response => {
        if (response.ok) {
            document.getElementById('bootloader').style.backgroundColor = 'orange';
        } else {
            alert('Failed to enter bootloader');
        }
    });
});

let map = L.map('map').setView([0, 0], 2);
L.tileLayer("https://{s}.tile.osm.org/{z}/{x}/{y}.png", {
    minZoom: 2,
    maxZoom: 21,
    tileSize: 256,
    zoomOffset: 0,
    errorTileUrl: '', // Prevents showing broken tile images
    keepBuffer: 2 // Keep extra tiles in memory
}).addTo(map);

// Store track points with their acknowledgment status
let trackPoints = [];
let trackSegments = [];  // Array of polylines with different colors
let marker = L.marker().setLatLng([0, 0]).addTo(map);
map.setView([0, 0], 15);

let auto_set_view = true;
let isFetchingGpsPos = false;

const MIN_MOVE_THRESHOLD = 0.0005;

function updateGpsPos() {
    if (isFetchingGpsPos) return;
    isFetchingGpsPos = true;
    fetch('/current_pos')
        .then(response => response.json())
        .then(data => {
            if (data.gps_fix_ok) {
                let lat = data.gps_lat;
                let lon = data.gps_lon;
                let newPoint = {
                    coords: [lat, lon],
                    acked: lastPacketWasAcked
                };
                trackPoints.push(newPoint);
                redrawTrack();
                marker.setLatLng([lat, lon]);
                if (auto_set_view) {
                    let current_map_center = map.getCenter();
                    if (Math.abs(current_map_center['lat'] - lat) > MIN_MOVE_THRESHOLD ||
                        Math.abs(current_map_center['lng'] - lon) > MIN_MOVE_THRESHOLD) {
                        map.setView([lat, lon]);
                    }
                }
            }
        })
        .catch(error => {
            console.error('Error fetching GPS position:', error);
        })
        .finally(() => {
            isFetchingGpsPos = false;
        });
}

function redrawTrack() {
    // Remove all existing segments
    trackSegments.forEach(segment => map.removeLayer(segment));
    trackSegments = [];

    if (trackPoints.length < 2) {
        // If we have only one point, draw a small circle marker
        if (trackPoints.length === 1) {
            let color = trackPoints[0].acked ? 'blue' : 'red';
            let segment = L.polyline([trackPoints[0].coords, trackPoints[0].coords], {
                color: color,
                weight: 3
            }).addTo(map);
            trackSegments.push(segment);
        }
        return;
    }

    // Draw segments between consecutive points with color based on destination point status
    for (let i = 0; i < trackPoints.length - 1; i++) {
        let color = trackPoints[i + 1].acked ? 'blue' : 'red';
        let segment = L.polyline(
            [trackPoints[i].coords, trackPoints[i + 1].coords],
            {
                color: color,
                weight: 3
            }
        ).addTo(map);
        trackSegments.push(segment);
    }
}

setInterval(updateStats, 1000);
updateStats();
setInterval(updateGpsPos, 1000);
updateGpsPos();

L.Control.Button = L.Control.extend({
    options: {
        position: 'topleft'
    },
    onAdd: function (map) {
        var container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');
        var button = L.DomUtil.create('a', 'leaflet-control-button', container);
        button.classList.add('active');
        L.DomEvent.disableClickPropagation(button);
        L.DomEvent.on(button, 'click', function () {
            auto_set_view = !auto_set_view;
            if (auto_set_view) {
                button.classList.add('active');
            } else {
                button.classList.remove('active');
            }
        });
        button.title = 'Toggle Auto-Center';
        button.innerHTML = '<svg xmlns="http://www.w3.org/2000/svg" class="icon icon-tabler icon-tabler-target" viewBox="0 0 24 24" stroke-width="1.5" stroke="currentColor" fill="none" stroke-linecap="round" stroke-linejoin="round"><path stroke="none" d="M0 0h24v24H0z" fill="none"/><circle cx="12" cy="12" r="9" /><circle cx="12" cy="12" r="5" /><circle cx="12" cy="12" r="1" /></svg>';
        return container;
    },
    onRemove: function (map) { },
});

var control = new L.Control.Button()
control.addTo(map);
