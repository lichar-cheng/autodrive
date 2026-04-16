from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path


if __name__ == "__main__":
    root = Path(__file__).resolve().parent
    handler = lambda *args, **kwargs: SimpleHTTPRequestHandler(*args, directory=str(root), **kwargs)
    httpd = ThreadingHTTPServer(("0.0.0.0", 8090), handler)
    print("Client running at http://0.0.0.0:8090")
    httpd.serve_forever()
