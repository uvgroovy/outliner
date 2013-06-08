import SimpleHTTPServer
import SocketServer
from multiprocessing import Process, Queue
from Queue import Empty
import sensor
import json

angleQ = Queue()

class MyHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):

    def __init__(self, *args):
       global angleQ
       self.queue = angleQ
       SimpleHTTPServer.SimpleHTTPRequestHandler.__init__(self, *args)

    def do_GET(self):
        """Serve a GET request."""
        if self.path != "/angles":
            return SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)
        else:
            angles = self.getAngles()
            self.send_response(200)
            self.send_header("Content-type", "text/json")
            self.end_headers()
            # long poll...
            self.wfile.write(json.dumps(angles))

    def getAngles(self):
        try:
            # if Q empty, do a long poll
            if self.queue.empty():
                return [self.queue.get(timeout = 5)]
                
            l = []
            while not self.queue.empty():
                l.append(self.queue.get())
            return l
                
        except Empty:
            return []
        
    def addAngle(self, a):
        self.queue.put(a)
    
def getSensorData(q):
    sensor.init()
    
    while True:
        q.put(sensor.getAngle())

PORT = 8000

# http://stackoverflow.com/questions/3911009/python-socketserver-baserequesthandler-knowing-the-port-and-use-the-port-already
class ReuseAddrServer(SocketServer.TCPServer):
    def __init__(self, (host, port), handler):
        SocketServer.TCPServer.__init__(self, (host, port), handler, bind_and_activate=False)
        self.allow_reuse_address = True
        self.server_bind()
        self.server_activate()

def main():
    handler = MyHandler
    server = SocketServer.TCPServer(("", PORT), handler)

    p = Process(target=getSensorData, args=(angleQ,))
    p.start()
    
    server.serve_forever()
    
    
    
if __name__ == "__main__":
    main()

