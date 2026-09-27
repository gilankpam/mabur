#!/usr/bin/env python3
# THROWAWAY SPIKE: static server with the COOP/COEP headers pthreads need.
# usage: serve.py [port=8808] [bind=127.0.0.1] [--tls]
# --tls serves HTTPS with tls/cert.pem + tls/key.pem (make them with
# ./mkcert.sh): WebUSB and SharedArrayBuffer need a secure context, which
# plain http is only on localhost -- so a phone on the LAN needs this.
import http.server, os, ssl, sys
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
srv = http.server.ThreadingHTTPServer((bind, port), H)
if tls:
    here = os.path.dirname(os.path.abspath(__file__))
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(os.path.join(here, 'tls/cert.pem'), os.path.join(here, 'tls/key.pem'))
    srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
print(f"serving {'https' if tls else 'http'}://{bind}:{port}/", flush=True)
srv.serve_forever()
