import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import folium
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer


class GpsDataRequestHandler(BaseHTTPRequestHandler):

    def __init__(self, node, *args, **kwargs):
        self.node = node
        super().__init__(*args, **kwargs)

    def do_GET(self):
        map_content = self.node.generate_map()

        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        # Send the Folium map HTML as the response
        self.wfile.write(map_content.encode())


class GPSDataSubscriber(Node):

    def __init__(self):
        super().__init__("gps_data_subscriber")
        self.subscription = self.create_subscription(
            NavSatFix, "gps", self.callback, 10
        )

        self.declare_parameter("port", 8080)
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        # Your node initialization code goes here
        self.history: list = []

        http_server_thread = threading.Thread(target=self.start_server)
        http_server_thread.daemon = True
        http_server_thread.start()
        pass

    def callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude

        if latitude != 0.0 and longitude != 0.0:  # Filter out invalid GPS data
            # Add the current GPS data point to the history
            self.history.append([latitude, longitude])
        pass

    def start_server(self):
        server_address = ("", self.port)
        httpd = HTTPServer(
            server_address,
            lambda *args, **kwargs: GpsDataRequestHandler(self, *args, **kwargs),
        )
        print(f"Starting visualizer at http://localhost:{self.port}")

        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            httpd.server_close()

    def generate_map(self):
        if len(self.history) > 0:
            [latitude, longitude] = self.history.pop()
            # Create a Folium map object
            map = folium.Map(location=[latitude, longitude], zoom_start=50)

            for lat, lon in self.history:
                folium.Marker(
                    [lat, lon], icon=folium.Icon(icon="cloud", color="blue")
                ).add_to(map)
        else:
            map = folium.Map()
        # Save the map to an HTML string
        map_html = map.get_root().render()

        return f"<!DOCTYPE html><html><head><title>GPS Location</title></head><body>{map_html}</body></html>"


def main(args=None):
    rclpy.init(args=args)
    gps_data_subscriber = GPSDataSubscriber()
    try:
        rclpy.spin(gps_data_subscriber)
    finally:
        gps_data_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
