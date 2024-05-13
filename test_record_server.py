from record_server import record_server

server = record_server(
    host = '127.0.0.1',
    port = 8091
)

server.run()