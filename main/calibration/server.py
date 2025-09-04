# server.py
# This program starts a simple HTTP server in Python that serves a single
# static image. The webpage will auto-refresh the image every 5 seconds.

import http.server
import socketserver
import os
import logging

# --- Configuration ---
PORT = 8080
IMAGE_PATH = "data/server_img.jpg"

# --- Set up logging ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')

class ImageRequestHandler(http.server.SimpleHTTPRequestHandler):
    """
    Custom request handler to serve an image that might be updated by another process.
    """

    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/':
            self.send_response(200)
            self.send_header("Content-type", "text/html; charset=utf-8")
            self.end_headers()
            # This HTML now includes an image tag and a script to refresh it.
            response_html = """
            <!DOCTYPE html>
            <html lang="en">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>Auto-Refreshing Image</title>
                <style>
                    body { font-family: sans-serif; text-align: center; margin-top: 50px; }
                    img { border: 2px solid #ccc; border-radius: 8px; max-width: 90%; }
                </style>
            </head>
            <body>
                <h1>Python Image Server</h1>
                <p>The image below will automatically refresh every 5 seconds.</p>
                <img id="refreshing-image" src="/image" alt="Dynamically updated image">

                <script>
                    document.addEventListener('DOMContentLoaded', (event) => {
                        const imageElement = document.getElementById('refreshing-image');
                        if (imageElement) {
                            setInterval(() => {
                                // By adding a timestamp query parameter, we force the browser
                                // to bypass its cache and request a fresh image from the server.
                                const timestamp = new Date().getTime();
                                imageElement.src = '/image?t=' + timestamp;
                                console.log('Image refreshed at ' + new Date().toLocaleTimeString());
                            }, 5000); // 5000 milliseconds = 5 seconds
                        }
                    });
                </script>
            </body>
            </html>
            """
            self.wfile.write(response_html.encode('utf-8'))

        # We use startswith because the path will be '/image?t=...'
        elif self.path.startswith('/image'):
            try:
                # Read the file from the disk on every single request.
                with open(IMAGE_PATH, 'rb') as f:
                    image_bytes = f.read()

                self.send_response(200)
                # Manually set the Content-Type header.
                self.send_header("Content-type", "image/png")
                # Add cache-control headers to prevent aggressive browser caching
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.end_headers()

                # Write the image data to the response.
                logging.info(f"Serving image to {self.client_address[0]}...")
                self.wfile.write(image_bytes)

            except FileNotFoundError:
                logging.error(f"Error: could not find image file '{IMAGE_PATH}'")
                error_message = "Image file not found."
                self.send_error(404, error_message)
            except Exception as e:
                logging.error(f"An unexpected error occurred: {e}")
                self.send_error(500, "Unable to load image due to a server error.")

        else:
            self.send_error(404, "File Not Found")

def run_server():
    """
    Initializes and runs the HTTP server.
    """
    # Create a TCP server that listens on the specified port and uses our custom handler.
    with socketserver.TCPServer(("", PORT), ImageRequestHandler) as httpd:
        logging.info(f"Starting server. Listening on http://localhost:{PORT}")
        logging.info(f"Assuming '{IMAGE_PATH}' exists in the same directory.")

        try:
            # Keep the server running until interrupted (e.g., Ctrl+C).
            httpd.serve_forever()
        except KeyboardInterrupt:
            logging.info("Server is shutting down.")
            httpd.server_close()

if __name__ == "__main__":
    run_server()
