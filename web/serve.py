#!/usr/bin/env python3
# Static server for the web GS page: the COOP/COEP headers pthreads need.
# Serves web/dist (built by `emcmake cmake -S web -B web/build-wasm && cmake
# --build web/build-wasm`, which copies web/www next to webgs.js/webgs.wasm).
# usage: serve.py [port=8808] [bind=127.0.0.1] [--tls]
# --tls serves HTTPS with web/tls/cert.pem + web/tls/key.pem (make them with
# ./mkcert.sh): WebUSB and SharedArrayBuffer need a secure context, which
# plain http is only on localhost -- so a phone on the LAN needs this.
import functools, http.server, os, ssl, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, 'dist')

class H(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cross-Origin-Opener-Policy', 'same-origin')
        self.send_header('Cross-Origin-Embedder-Policy', 'require-corp')
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()
    def log_message(self, *a): pass

args = [a for a in sys.argv[1:] if a != '--tls']
tls = '--tls' in sys.argv
port = int(args[0]) if len(args) > 0 else 8808
bind = args[1] if len(args) > 1 else '127.0.0.1'
handler = functools.partial(H, directory=ROOT)
srv = http.server.ThreadingHTTPServer((bind, port), handler)
if tls:
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(os.path.join(HERE, 'tls/cert.pem'), os.path.join(HERE, 'tls/key.pem'))
    # Handshake lazily, on the handler thread's first read: done in accept()
    # (the default) one idle or speculative client connection that never
    # handshakes blocks the only accepting thread, and the page hangs.
    srv.socket = ctx.wrap_socket(srv.socket, server_side=True, do_handshake_on_connect=False)
print(f"serving {'https' if tls else 'http'}://{bind}:{port}/  (root {ROOT})", flush=True)
srv.serve_forever()
