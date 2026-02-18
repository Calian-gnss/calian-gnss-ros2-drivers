"""GPS Visualizer — serves a live Folium map of the antenna's position.

Subscribes to the ``gps`` topic and renders location history on an HTTP
endpoint (default port 8080).
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import folium
from http.server import BaseHTTPRequestHandler, HTTPServer


class _GpsDataRequestHandler(BaseHTTPRequestHandler):
    """HTTP handler that returns the current Folium map HTML."""

    def __init__(self, node, *args, **kwargs):
        self.node = node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        map_content = self.node.generate_map()
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(map_content.encode())

    def log_message(self, format, *args):
        """Suppress default stderr logging from BaseHTTPRequestHandler."""


class GPSDataSubscriber(Node):
    """ROS 2 node that subscribes to ``gps`` and serves a map over HTTP."""

    def __init__(self):
        super().__init__("gps_data_subscriber")
        self.subscription = self.create_subscription(
            NavSatFix, "gps", self._gps_callback, 10
        )

        self.declare_parameter("port", 8080)
        self.port: int = (
            self.get_parameter("port").get_parameter_value().integer_value
        )

        self._history_lock = threading.Lock()
        self.location_history: list[list[float]] = []

        http_thread = threading.Thread(target=self._start_server, daemon=True)
        http_thread.start()

    def _gps_callback(self, msg: NavSatFix) -> None:
        """Append valid GPS coordinates to the history (thread-safe)."""
        if msg.latitude != 0.0 and msg.longitude != 0.0:
            with self._history_lock:
                self.location_history.append([msg.latitude, msg.longitude])

    def _start_server(self) -> None:
        """Run the HTTP server in a background thread."""
        httpd = HTTPServer(
            ("", self.port),
            lambda *args, **kwargs: _GpsDataRequestHandler(self, *args, **kwargs),
        )
        self.get_logger().info(
            f"Starting visualizer at http://localhost:{self.port}"
        )
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            httpd.server_close()

    def generate_map(self) -> str:
        """Build the Folium map HTML from the current location history."""
        with self._history_lock:
            history_snapshot = list(self.location_history)

        if history_snapshot:
            latitude, longitude = history_snapshot[-1]
            gps_map = folium.Map(location=[latitude, longitude], zoom_start=50)
            for lat, lon in history_snapshot[:-1]:
                folium.Marker(
                    [lat, lon], icon=folium.Icon(icon="cloud", color="blue")
                ).add_to(gps_map)
        else:
            gps_map = folium.Map()

        map_html = gps_map.get_root().render()
        return (
            "<!DOCTYPE html><html><head><title>GPS Location</title></head>"
            f"<body>{map_html}</body></html>"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GPSDataSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
