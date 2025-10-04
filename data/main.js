let combinedChart = null;

function updateCombinedChart(rssi, dist) {
    const length = rssi.length;
    const labels = Array(length).fill(0).map((_, i) => i - (length - 1));

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
                        pointRadius: 0,
                        tension: 0.19
                    },
                    {
                        label: 'Distance (m)',
                        data: dist,
                        borderColor: '#2563ea',
                        backgroundColor: 'rgba(37,99,234,0.09)',
                        fill: false,
                        yAxisID: 'y_dist',
                        pointRadius: 0,
                        tension: 0.17
                    }
                ]
            },
            options: {
                responsive: true,
                scales: {
                    x: {
                        display: false
                    },
                    y_rssi: {
                        type: 'linear',
                        position: 'left',
                        title: {
                            display: true,
                            text: 'RSSI (dBm)'
                        },
                        ticks: {
                            // example range - adjust as needed
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
                        // grid lines only on left axis
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
function updateStats() {
    if (isFetchingStats) return;  // Skip if fetch in progress
    isFetchingStats = true;
    fetch('stats.json')
        .then(response => response.json())
        .then(data => {
            document.getElementById('sent').textContent = data.packets_sent;
            document.getElementById('acked').textContent = data.packets_acked;
            document.getElementById('lost_pct').textContent = data.loss_pct.toFixed(2);
            document.getElementById('rssi').textContent = data.last_rssi;
            document.getElementById('distance').textContent = data.distance.toFixed(1);
            document.getElementById('gps_numSV').textContent = data.gps_numSV;
            document.getElementById('gps_lat').textContent = data.gps_lat.toFixed(7);
            document.getElementById('gps_lon').textContent = data.gps_lon.toFixed(7);
            document.getElementById('gps_utc').textContent = data.gps_utc_time;
            document.getElementById('gps_fix_type').textContent = data.gps_fix_type;
            document.getElementById('gps_fix_ok').textContent = data.gps_fix_ok;
            updateCombinedChart(data.rssi_hist || [], data.distance_hist || []);
        })
        .catch(error => {
            console.error('Error fetching stats:', error);
        })
        .finally(() => {
            isFetchingStats = false;  // Reset flag regardless of success/failure
        });
}

document.getElementById('resetBtn').addEventListener('click', () => {
    fetch('/reset_stats', { method: 'POST' });
});

document.getElementById('ota').addEventListener('click', () => {
    // fetch and check 200 status
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
}).addTo(map);

let trackPoints = [];
let trackLine = L.polyline(trackPoints, { color: 'blue' }).addTo(map);
let marker = L.marker().setLatLng([0, 0]).addTo(map);
map.setView([0, 0], 15);

let auto_set_view = true;
let isFetchingGpsPos = false;
function updateGpsPos() {
    if (isFetchingGpsPos) return;  // Skip if fetch in progress
    isFetchingGpsPos = true;
    fetch('/current_pos')
        .then(response => response.json())
        .then(data => {
            if (data.gps_fix_ok) {
                let lat = data.gps_lat;
                let lon = data.gps_lon;
                trackPoints.push([lat, lon]);
                trackLine.setLatLngs(trackPoints);
                marker.setLatLng([lat, lon]);
                if (auto_set_view) {
                    map.setView([lat, lon]);
                }
            }
        })
        .catch(error => {
            console.error('Error fetching GPS position:', error);
        })
        .finally(() => {
            isFetchingGpsPos = false;  // Reset flag regardless of outcome
        });
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
