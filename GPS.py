import serial
import gps

# Setup GPS
gpsd = gps.gps(mode=gps.WATCH_ENABLE)

def get_current_position():
    # Fetch GPS data
    report = gpsd.next()
    if report['class'] == 'TPV':
        if hasattr(report, 'lat') and hasattr(report, 'lon'):
            return report.lat, report.lon
    return None, None
