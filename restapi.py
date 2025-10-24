import requests
import datetime as dt

from environments import IOT_STATION_KEY, SERVER_URL, SERVER_PORT
from environments import OPENMETEO_URL, OPENMETEO_LATITUDE, OPENMETEO_LONGITUDE, OPENMETEO_LIMIT

last_updated = None
current_weather = {}

def stream_to_server(data):
    url = "http://{0}:{1}/api/sensors".format(SERVER_URL, SERVER_PORT)
    headers = {
        "Content-Type": "application/json",
        "IOT-Station-Key": IOT_STATION_KEY
    }

    response = requests.post(url = url, headers = headers, json = data)

    if response.status_code == 200:
        return response.json()
    else:
        return {}

def update_weather_data():
    if (dt.datetime.now() - last_updated).total_seconds() > OPENMETEO_LIMIT or last_updated == None:
        url = OPENMETEO_URL
        params = {
            "latitude": OPENMETEO_LATITUDE,
            "longitude": OPENMETEO_LONGITUDE,
            "current": ["temperature_2m", "relative_humidity_2m", "wind_speed_10m", "winddirection_10m"]
        }

        response = requests.get(url = url, params = params)

        if response.status_code == 200:
            last_updated = dt.datetime.now()
            current_weather = response.json()
        else:
            pass
    else:
        pass

    return current_weather