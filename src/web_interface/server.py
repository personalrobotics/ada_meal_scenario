import SimpleHTTPServer
import SocketServer
import sys
import threading
import logging
import morsel_backend
import feed_listener

logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)

# start the feed listener
listenerThread = threading.Thread(target=feed_listener.MorselFeedListener.init)
listenerThread.daemon = True
listenerThread.start()

# start the server
PORT = 8000
Handler = morsel_backend.MorselRequestHandler

SocketServer.TCPServer.allow_reuse_address = True
httpd = SocketServer.TCPServer(("", PORT), Handler)

print "serving at port", PORT
httpd.serve_forever()