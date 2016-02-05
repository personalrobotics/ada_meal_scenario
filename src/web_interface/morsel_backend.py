import SimpleHTTPServer
import cgi
import json
import logging
import feed_listener

class MorselRequestHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):

    def do_POST(self):
        path = self.path
        logging.debug(path)
        if path == "/register_click":
            content_len = int(self.headers.getheader('content-length'))
            query_string = self.rfile.read(content_len)
            query = cgi.parse_qs(query_string)
            if 'i' in query:
                logging.info("selected morsel " + query['i'][0])
                feed_listener.MorselFeedListener.publish_selection(query['i'][0])
                self.send_response(200)
                self.send_header("Content-type", "text/plain")
                self.end_headers()
                self.wfile.write("OK")
            else:
                logging.info("invalid register_click request")
                self.send_response(400)

        elif path == "/serve_morsels":
            logging.info("serving morsels")
            # morsels = '[{"x":300, "y":400, "r":70},{"x":500, "y":800, "r":85},{"x":1000, "y":900, "r":60}]'
            morsels = feed_listener.MorselFeedListener.bites
            logging.debug(json.dumps(morsels))
            self.send_response(200)
            self.send_header("Content-type", "text/json")
            self.end_headers()
            self.wfile.write(morsels)

        else:
            logging.info("unknown request path")
            self.send_response(501)


    def do_GET(self):
        SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)

    def pushUpdate(self, morsels):
        logging.debug(json.dumps(morsels))
        self.send_response(200)
        self.send_header("Content-type", "text/event-stream")
        self.end_headers()
        self.wfile.write("data: " + morsels + "\n\n")
