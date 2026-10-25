"""Hack-IO-2026 Sim desktop launcher: serves the flight sim locally and opens it in your browser.

Runs offline (three.js and the splat viewer are bundled). Scenes are folders made by
splat/build_scene.py; put them in a "scenes" folder next to HackIO-Sim.exe (or next to
this script) and pick them from the Scene menu in the page.

  HackIO-Sim.exe                 # open the first scene
  HackIO-Sim.exe tuttle-park     # open a specific scene
  HackIO-Sim.exe --port 8800
"""
import argparse, http.server, json, os, socket, socketserver, sys, threading, webbrowser
from urllib.parse import unquote, urlparse

CDN = {"https://cdn.jsdelivr.net/npm/three@0.167.1/build/three.module.js": "/vendor/three.module.js",
       "https://cdn.jsdelivr.net/npm/three@0.167.1/examples/jsm/": "/vendor/addons/",
       "https://cdn.jsdelivr.net/npm/@mkkellogg/gaussian-splats-3d@0.4.7/build/gaussian-splats-3d.module.js":
           "/vendor/gaussian-splats-3d.module.js"}


def bundle_dir():
    return getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))


def app_dir():   # folder of the .exe (frozen) or of this script
    return os.path.dirname(sys.executable if getattr(sys, "frozen", False) else os.path.abspath(__file__))


def scene_roots():
    roots = [os.path.join(app_dir(), "scenes"), os.path.join(os.getcwd(), "scenes"),
             os.path.join(bundle_dir(), "scenes"),
             os.path.join(os.path.dirname(os.path.dirname(app_dir())), "scenes")]   # repo checkout
    seen, out = set(), []
    for r in roots:
        r = os.path.normpath(r)
        if r not in seen and os.path.isdir(r):
            seen.add(r); out.append(r)
    return out


def list_scenes():
    names = []
    for root in scene_roots():
        for n in sorted(os.listdir(root)):
            if os.path.exists(os.path.join(root, n, "scene_splat.b64.txt")) and n not in names:
                names.append(n)
    return names


def find_scene_file(name, rel):
    for root in scene_roots():
        p = os.path.normpath(os.path.join(root, name, rel))
        if p.startswith(os.path.normpath(os.path.join(root, name))) and os.path.isfile(p):
            return p
    return None


LATEST_POSE = {"pose": None}   # posted by hackio_sim.Sim(viz=...) scripts


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=os.path.join(bundle_dir(), "web"), **kw)

    def log_message(self, *a):
        pass

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = unquote(urlparse(self.path).path)
        if path in ("/", "/index.html"):
            html = open(os.path.join(bundle_dir(), "web", "index.html"), encoding="utf-8").read()
            for k, v in CDN.items():
                html = html.replace(k, v)
            page = ("<!doctype html><html><head><meta charset='utf-8'>"
                    "<meta name='viewport' content='width=device-width,initial-scale=1,viewport-fit=cover'>"
                    "</head><body>" + html + "</body></html>")
            return self._send(page.encode("utf-8"), "text/html; charset=utf-8")
        if path == "/api/pose":
            return self._send(json.dumps(LATEST_POSE["pose"]).encode(), "application/json")
        if path == "/scenes/index.json":
            return self._send(json.dumps(list_scenes()).encode(), "application/json")
        if path.startswith("/scenes/"):
            parts = path.split("/", 3)
            f = find_scene_file(parts[2], parts[3]) if len(parts) == 4 else None
            if not f:
                self.send_error(404); return
            ctype = "application/json" if f.endswith(".json") else "text/plain"
            return self._send(open(f, "rb").read(), ctype)
        return super().do_GET()


def _post(self):
    if urlparse(self.path).path != "/api/pose":
        self.send_error(404); return
    n = int(self.headers.get("Content-Length", 0))
    try:
        LATEST_POSE["pose"] = json.loads(self.rfile.read(n) or b"null")
    except ValueError:
        self.send_error(400); return
    self._send(b"ok", "text/plain")


Handler.do_POST = _post


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", nargs="?")
    ap.add_argument("--port", type=int, default=8800, help="0 = any free port")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    scenes = list_scenes()
    if not scenes:
        print("No scenes found. Put scene folders (made by build_scene.py) in:", os.path.join(app_dir(), "scenes"))
    port = a.port
    with socket.socket() as s:   # fall back to a free port if 8800 is taken
        try:
            s.bind(("127.0.0.1", port))
        except OSError:
            s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", port), Handler)
    srv.daemon_threads = True
    name = a.scene or (scenes[0] if scenes else "")
    url = f"http://127.0.0.1:{port}/#{name}"
    print(f"Scripts can stream to this window: Sim(..., viz='http://127.0.0.1:{port}')")
    print(f"Hack-IO-2026 Sim running at {url}\nScenes: {', '.join(scenes) or 'none'}\nClose this window to stop.")
    if not a.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
