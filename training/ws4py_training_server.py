from wsgiref.simple_server import make_server
# from ws4py.websocket import WebSocket
from ws4py.websocket import EchoWebSocket
from ws4py.server.wsgirefserver import WSGIServer, WebSocketWSGIRequestHandler
from ws4py.server.wsgiutils import WebSocketWSGIApplication

# class EchoWebSocket(WebSocket):
#   def received_message(self, message):
#     self.send(message.data, message.is_binary)

server = make_server('', 9000, server_class=WSGIServer,
  handler_class=WebSocketWSGIRequestHandler,
  app=WebSocketWSGIApplication(handler_cls=EchoWebSocket))
server.initialize_websockets_manager()
server.serve_forever()
