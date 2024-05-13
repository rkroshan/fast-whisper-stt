from record_server import record_server
from record_client import CommMethod

server = record_server(
    host = '127.0.0.1',
    port = 8091,
    communicationMethod=CommMethod.NPARRAY
)

server.run()