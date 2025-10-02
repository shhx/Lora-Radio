function updateStats() {
    fetch('stats.json')
        .then(response => response.json())
        .then(data => {
            document.getElementById('sent').textContent = data.packets_sent;
            document.getElementById('acked').textContent = data.packets_acked;
            document.getElementById('rssi').textContent = data.last_rssi;
            document.getElementById('distance').textContent = data.distance.toFixed(1);
            document.getElementById('lastack').textContent = data.last_ack;
            document.getElementById('lastsent').textContent = data.last_sent;

            document.getElementById('gps_lat').textContent = data.gps_lat.toFixed(7);
            document.getElementById('gps_lon').textContent = data.gps_lon.toFixed(7);
            document.getElementById('gps_numSV').textContent = data.gps_numSV;
            document.getElementById('gps_utc').textContent = data.gps_utc_time;
            document.getElementById('gps_fix_type').textContent = data.gps_fix_type;
            document.getElementById('gps_fix_ok').textContent = data.gps_fix_ok;
        });
}
setInterval(updateStats, 1000);
updateStats();
